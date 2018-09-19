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

#[derive(Debug)]
pub struct Channel {
    type_: ChannelType,
    values: Vec<f32>,
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
    let mut ret = Mocap {
        num_frames: bvh.motion.num_frames,
        frame_time: bvh.motion.frame_time as _,
        root: build_joint(&bvh.hierarchy.root),
    };

    let mut frame_index = 0;
    let mut frame_data_index = 0;
    for _ in 0..bvh.motion.num_frames {
        pull_frame_data(&mut ret.root, &bvh.motion.frames, &mut frame_index, &mut frame_data_index);
    }

    ret
}

fn build_joint(bvh_joint: &bvh::Joint) -> Joint {
    Joint {
        name: bvh_joint.name.clone(),
        offset: (bvh_joint.offset.x as _, bvh_joint.offset.y as _, bvh_joint.offset.z as _),
        channels: bvh_joint.channels.iter().map(|channel| Channel {
            type_: match channel {
                bvh::Channel::XPosition => ChannelType::TranslationX,
                bvh::Channel::YPosition => ChannelType::TranslationY,
                bvh::Channel::ZPosition => ChannelType::TranslationZ,
                bvh::Channel::XRotation => ChannelType::RotationX,
                bvh::Channel::YRotation => ChannelType::RotationY,
                bvh::Channel::ZRotation => ChannelType::RotationZ,
            },
            values: Vec::new(),
        }).collect(),
        children: match bvh_joint.children {
            bvh::JointChildren::Joints(ref bvh_joints) => JointChildren::Joints(bvh_joints.iter().map(build_joint).collect()),
            bvh::JointChildren::EndSite(ref bvh_end_site) => JointChildren::EndSite((bvh_end_site.offset.x as _, bvh_end_site.offset.y as _, bvh_end_site.offset.z as _)),
        },
    }
}

fn pull_frame_data(joint: &mut Joint, frames: &Vec<Vec<f64>>, frame_index: &mut usize, frame_data_index: &mut usize) {
    for channel in joint.channels.iter_mut() {
        channel.values.push(frames[*frame_index][*frame_data_index] as _);
        *frame_data_index += 1;
        if *frame_data_index >= frames[*frame_index].len() {
            *frame_data_index = 0;
            *frame_index += 1;
        }
    }

    if let JointChildren::Joints(ref mut joints) = joint.children {
        for joint in joints.iter_mut() {
            pull_frame_data(joint, frames, frame_index, frame_data_index);
        }
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

        for (index, value) in channel.values.iter().enumerate() {
            frames[index].push(*value as _);
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

fn dump_channels<W: Write>(joint: &Joint, w: &mut W) -> io::Result<()> {
    for channel in joint.channels.iter() {
        for (index, value) in channel.values.iter().enumerate() {
            writeln!(w, "{};{}", index, value)?;
        }
    }

    if let JointChildren::Joints(ref joints) = joint.children {
        for joint in joints.iter() {
            dump_channels(joint, w)?;
        }
    }

    Ok(())
}

fn main() {
    let input_file_name = args().nth(1).unwrap();
    let output_file_name = args().nth(2).unwrap();
    let csv_file_name = args().nth(3).unwrap();

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
        dump_channels(&mocap.root, &mut csv).unwrap();
    }
}
