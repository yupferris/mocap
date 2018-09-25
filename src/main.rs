extern crate bvh;

use std::env::args;
use std::fs::File;
use std::io::{self, Read, Write};

#[derive(Debug)]
struct Mocap {
    num_frames: u32,
    frame_time: f32,
    channel_quantization_bits: u8, // Must be in [1, 8]
    root: Joint,
}

#[derive(Debug)]
struct Joint {
    name: String,
    offset: (f32, f32, f32),
    channels: Vec<Channel>,
    children: JointChildren,
}

#[derive(Debug)]
pub struct Channel {
    type_: ChannelType,
    value_range_min: f32,
    value_range: f32,
    deltas: Vec<i8>,
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

fn build_mocap(bvh: &bvh::Bvh, channel_quantization_bits: u8) -> Mocap {
    let mut channel_index = 0;

    Mocap {
        num_frames: bvh.motion.num_frames,
        frame_time: bvh.motion.frame_time as _,
        channel_quantization_bits: channel_quantization_bits,
        root: build_joint(&bvh.hierarchy.root, &bvh.motion.frames, &mut channel_index, channel_quantization_bits),
    }
}

fn build_joint(bvh_joint: &bvh::Joint, frames: &Vec<Vec<f64>>, channel_index: &mut usize, channel_quantization_bits: u8) -> Joint {
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
            (((value - value_range_min) / value_range) * (((1 << channel_quantization_bits) - 1) as f64)) as u8
        } else {
            0
        }).collect::<Vec<_>>();

        let mut deltas = Vec::with_capacity(values.len());
        let mut previous_value = 0;
        for value in values.iter() {
            let value = *value;

            let delta = (value as i8) - (previous_value as i8);
            deltas.push(delta);

            previous_value = value;
        }

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
            deltas: deltas,
        });

        *channel_index += 1;
    }

    Joint {
        name: bvh_joint.name.clone(),
        offset: (bvh_joint.offset.x as _, bvh_joint.offset.y as _, bvh_joint.offset.z as _),
        channels: channels,
        children: match bvh_joint.children {
            bvh::JointChildren::Joints(ref bvh_joints) => JointChildren::Joints(bvh_joints.iter().map(|joint| build_joint(joint, frames, channel_index, channel_quantization_bits)).collect()),
            bvh::JointChildren::EndSite(ref bvh_end_site) => JointChildren::EndSite((bvh_end_site.offset.x as _, bvh_end_site.offset.y as _, bvh_end_site.offset.z as _)),
        },
    }
}

fn build_bvh(mocap: &Mocap) -> bvh::Bvh {
    let mut frames = vec![Vec::new(); mocap.num_frames as usize];

    bvh::Bvh {
        hierarchy: bvh::Hierarchy {
            root: build_bvh_joint(&mocap.root, &mut frames, mocap.channel_quantization_bits),
        },
        motion: bvh::Motion {
            num_frames: mocap.num_frames,
            frame_time: mocap.frame_time as _,
            frames: frames,
        },
    }
}

fn build_bvh_joint(joint: &Joint, frames: &mut Vec<Vec<f64>>, channel_quantization_bits: u8) -> bvh::Joint {
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

        let mut previous_value = 0;
        for (index, delta) in channel.deltas.iter().enumerate() {
            let delta = *delta;

            let value = ((previous_value as i8) + (delta as i8)) as u8;
            frames[index].push((channel.value_range_min as f64) + ((value as f64) / (((1 << channel_quantization_bits) - 1) as f64)) * (channel.value_range as f64));

            previous_value = value;
        }
    }

    bvh::Joint {
        name: joint.name.clone(),
        offset: build_bvh_offset(&joint.offset),
        channels: channels,
        children: match joint.children {
            JointChildren::Joints(ref joints) => bvh::JointChildren::Joints(joints.iter().map(|joint| build_bvh_joint(joint, frames, channel_quantization_bits)).collect()),
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
    let mocap = build_mocap(&bvh, 8);
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
        for (index, delta) in channel.deltas.iter().enumerate() {
            writeln!(w, "{};{}", index, delta)?;
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
        for delta in channel.deltas.iter() {
            w.write_all(&[*delta as u8])?;
        }
    }

    if let JointChildren::Joints(ref joints) = joint.children {
        for joint in joints.iter() {
            dump_channels_raw(joint, w)?;
        }
    }

    Ok(())
}
