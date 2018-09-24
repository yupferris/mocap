extern crate bvh;

use std::env::args;
use std::fs::File;
use std::io::{self, Read, Write};

#[derive(Debug)]
struct Mocap {
    num_frames: u32,
    frame_time: f32,
    root: Joint,
}

#[derive(Debug)]
struct Joint {
    name: String,
    offset: (f32, f32, f32),
    channels: Vec<Channel>,
    children: JointChildren,
}

const CHANNEL_QUANTIZATION_BITS: usize = 7; // Must be in [1, 16]

const CHANNEL_LPC_ORDER: usize = 1; // Must be >= 1
const CHANNEL_LPC_COEFFICIENT_BITS: usize = 8; // Must be in [1, 8]

#[derive(Debug)]
pub struct Channel {
    type_: ChannelType,
    value_range_min: f32,
    value_range: f32,
    lpc_coefficients: [u8; CHANNEL_LPC_ORDER],
    residuals: Vec<i16>,
}

#[derive(Debug)]
pub enum ChannelType {
    TranslationX,
    TranslationY,
    TranslationZ,
    RotationX,
    RotationY,
    RotationZ,
}

#[derive(Debug)]
enum JointChildren {
    Joints(Vec<Joint>),
    EndSite((f32, f32, f32)),
}

fn build_mocap(bvh: &bvh::Bvh) -> Mocap {
    let mut channel_index = 0;

    Mocap {
        num_frames: bvh.motion.num_frames,
        frame_time: bvh.motion.frame_time as _,
        root: build_joint(&bvh.hierarchy.root, &bvh.motion.frames, &mut channel_index),
    }
}

fn build_joint(bvh_joint: &bvh::Joint, frames: &Vec<Vec<f64>>, channel_index: &mut usize) -> Joint {
    let mut channels = Vec::new();
    for channel in bvh_joint.channels.iter() {
        let mut values = Vec::new();
        for frame in frames.iter() {
            values.push(frame[*channel_index]);
        }
        let mut value_range_min = values[0];
        let mut value_range_max = values[0];
        for value in values.iter() {
            if *value < value_range_min {
                value_range_min = *value;
            }
            if *value > value_range_max {
                value_range_max = *value;
            }
        }
        let mut value_range = value_range_max - value_range_min;
        let values = values.iter().map(|value| if value_range > 0.0 {
            (((value - value_range_min) / value_range) * (((1 << CHANNEL_QUANTIZATION_BITS) - 1) as f64)) as u16
        } else {
            0
        }).collect::<Vec<_>>();

        let mut best = None;
        for packed_coefficients in 0..(1 << (CHANNEL_LPC_ORDER * CHANNEL_LPC_COEFFICIENT_BITS)) {
            let mut coefficients = [0; CHANNEL_LPC_ORDER];
            for coefficient_index in 0..CHANNEL_LPC_ORDER {
                coefficients[coefficient_index] = ((packed_coefficients >> (coefficient_index * CHANNEL_LPC_COEFFICIENT_BITS)) & ((1 << CHANNEL_LPC_COEFFICIENT_BITS) - 1)) as u8;
            }

            let mut previous_values = [0; CHANNEL_LPC_ORDER];

            let mut error = 0;
            let mut residuals = Vec::with_capacity(values.len());
            for value in values.iter() {
                let value = *value;

                let mut prediction = 0;
                for coefficient_index in 0..CHANNEL_LPC_ORDER {
                    let sign_extended_coefficient = ((coefficients[coefficient_index] << (8 - CHANNEL_LPC_COEFFICIENT_BITS)) as i8) >> ((8 - CHANNEL_LPC_COEFFICIENT_BITS));
                    prediction += ((previous_values[coefficient_index] as i32) * (sign_extended_coefficient as i32)) >> (CHANNEL_LPC_COEFFICIENT_BITS - 1);
                }

                let residual = (value as i16) - (prediction as i16);
                residuals.push(residual);

                error += (residual as i64) * (residual as i64);

                let mut coefficient_index = CHANNEL_LPC_ORDER - 1;
                while coefficient_index > 0 {
                    previous_values[coefficient_index] = previous_values[coefficient_index - 1];
                    coefficient_index -= 1;
                }
                previous_values[0] = value;
            }

            best = match best {
                Some((best_error, best_coefficients, best_residuals)) => {
                    if error < best_error {
                        Some((error, coefficients, residuals))
                    } else {
                        Some((best_error, best_coefficients, best_residuals))
                    }
                }
                None => Some((error, coefficients, residuals)),
            };
        }
        let best = best.unwrap();

        println!("Best coefficients: {:?} (error: {})", best.1.iter().map(|coefficient| {
            ((coefficient << (8 - CHANNEL_LPC_COEFFICIENT_BITS)) as i8) >> ((8 - CHANNEL_LPC_COEFFICIENT_BITS))
        }).collect::<Vec<_>>(), best.0);

        channels.push(Channel {
            type_: match channel {
                bvh::Channel::XPosition => ChannelType::TranslationX,
                bvh::Channel::YPosition => ChannelType::TranslationY,
                bvh::Channel::ZPosition => ChannelType::TranslationZ,
                bvh::Channel::XRotation => ChannelType::RotationX,
                bvh::Channel::YRotation => ChannelType::RotationY,
                bvh::Channel::ZRotation => ChannelType::RotationZ,
            },
            value_range_min: value_range_min as _,
            value_range: value_range as _,
            lpc_coefficients: best.1,
            residuals: best.2,
        });

        *channel_index += 1;
    }

    Joint {
        name: bvh_joint.name.clone(),
        offset: (bvh_joint.offset.x as _, bvh_joint.offset.y as _, bvh_joint.offset.z as _),
        channels: channels,
        children: match bvh_joint.children {
            bvh::JointChildren::Joints(ref bvh_joints) => JointChildren::Joints(bvh_joints.iter().map(|joint| build_joint(joint, frames, channel_index)).collect()),
            bvh::JointChildren::EndSite(ref bvh_end_site) => JointChildren::EndSite((bvh_end_site.offset.x as _, bvh_end_site.offset.y as _, bvh_end_site.offset.z as _)),
        },
    }
}

fn build_bvh(mocap: &Mocap) -> bvh::Bvh {
    let mut frames = vec![Vec::new(); mocap.num_frames as usize];

    bvh::Bvh {
        hierarchy: bvh::Hierarchy {
            root: build_bvh_joint(&mocap.root, &mut frames),
        },
        motion: bvh::Motion {
            num_frames: mocap.num_frames,
            frame_time: mocap.frame_time as _,
            frames: frames,
        },
    }
}

fn build_bvh_joint(joint: &Joint, frames: &mut Vec<Vec<f64>>) -> bvh::Joint {
    let mut channels = Vec::new();
    for channel in joint.channels.iter() {
        channels.push(match channel.type_ {
            ChannelType::TranslationX => bvh::Channel::XPosition,
            ChannelType::TranslationY => bvh::Channel::YPosition,
            ChannelType::TranslationZ => bvh::Channel::ZPosition,
            ChannelType::RotationX => bvh::Channel::XRotation,
            ChannelType::RotationY => bvh::Channel::YRotation,
            ChannelType::RotationZ => bvh::Channel::ZRotation,
        });

        let mut previous_values = [0; CHANNEL_LPC_ORDER];
        for (index, residual) in channel.residuals.iter().enumerate() {
            let residual = *residual;

            let mut prediction = 0;
            for coefficient_index in 0..CHANNEL_LPC_ORDER {
                let sign_extended_coefficient = ((channel.lpc_coefficients[coefficient_index] << (8 - CHANNEL_LPC_COEFFICIENT_BITS)) as i8) >> ((8 - CHANNEL_LPC_COEFFICIENT_BITS));
                prediction += ((previous_values[coefficient_index] as i32) * (sign_extended_coefficient as i32)) >> (CHANNEL_LPC_COEFFICIENT_BITS - 1);
            }

            let value = ((prediction as i16) + (residual as i16)) as u16;
            frames[index].push((channel.value_range_min as f64) + ((value as f64) / (((1 << CHANNEL_QUANTIZATION_BITS) - 1) as f64)) * (channel.value_range as f64));

            let mut coefficient_index = CHANNEL_LPC_ORDER - 1;
            while coefficient_index > 0 {
                previous_values[coefficient_index] = previous_values[coefficient_index - 1];
                coefficient_index -= 1;
            }
            previous_values[0] = value;
        }
    }

    bvh::Joint {
        name: joint.name.clone(),
        offset: build_bvh_offset(&joint.offset),
        channels: channels,
        children: match joint.children {
            JointChildren::Joints(ref joints) => bvh::JointChildren::Joints(joints.iter().map(|joint| build_bvh_joint(joint, frames)).collect()),
            JointChildren::EndSite(ref offset) => bvh::JointChildren::EndSite(bvh::EndSite {
                offset: build_bvh_offset(offset),
            }),
        },
    }
}

fn build_bvh_offset(offset: &(f32, f32, f32)) -> bvh::Offset {
    bvh::Offset {
        x: offset.0 as _,
        y: offset.1 as _,
        z: offset.2 as _,
    }
}

fn main() {
    let input_file_name = args().nth(1).unwrap();
    let output_file_name = args().nth(2).unwrap();
    let csv_file_name = args().nth(3).unwrap();
    let raw_file_name = args().nth(4).unwrap();

    let input = {
        let mut ret = String::new();
        let mut file = File::open(input_file_name).unwrap();
        file.read_to_string(&mut ret).unwrap();
        ret
    };

    let bvh = bvh::parse(&input).unwrap();
    let mocap = build_mocap(&bvh);
    //println!("Result: {:#?}", mocap);

    {
        let bvh = build_bvh(&mocap);
        let mut output = File::create(output_file_name).unwrap();
        bvh::serialize(&bvh, &mut output).unwrap();
    }

    {
        let mut csv = File::create(csv_file_name).unwrap();
        dump_channels_csv(&mocap.root, &mut csv).unwrap();
    }

    {
        let mut raw = File::create(raw_file_name).unwrap();
        dump_channels_raw(&mocap.root, &mut raw).unwrap();
    }
}

fn dump_channels_csv<W: Write>(joint: &Joint, w: &mut W) -> io::Result<()> {
    for channel in joint.channels.iter() {
        for (index, residual) in channel.residuals.iter().enumerate() {
            writeln!(w, "{};{}", index, residual)?;
        }
    }

    if let JointChildren::Joints(ref joints) = joint.children {
        for joint in joints.iter() {
            dump_channels_csv(joint, w)?;
        }
    }

    Ok(())
}

fn dump_channels_raw<W: Write>(joint: &Joint, w: &mut W) -> io::Result<()> {
    for channel in joint.channels.iter() {
        for residual in channel.residuals.iter() {
            w.write_all(&[(residual >> 0) as u8, (residual >> 8) as u8])?;
        }
    }

    if let JointChildren::Joints(ref joints) = joint.children {
        for joint in joints.iter() {
            dump_channels_raw(joint, w)?;
        }
    }

    Ok(())
}
