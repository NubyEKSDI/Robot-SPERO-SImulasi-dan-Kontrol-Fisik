#!/usr/bin/env python3

import xml.etree.ElementTree as ET
import re
import os

def scale_value(value, factor):
    """Scale a numeric value by the given factor."""
    try:
        return str(float(value) * factor)
    except ValueError:
        return value

def scale_pose(pose_str, factor):
    """Scale all numeric values in a pose string."""
    values = pose_str.split()
    scaled_values = [scale_value(v, factor) for v in values]
    return ' '.join(scaled_values)

def scale_size(size_str, factor):
    """Scale all numeric values in a size string."""
    values = size_str.split()
    scaled_values = [scale_value(v, factor) for v in values]
    return ' '.join(scaled_values)

def process_sdf_file(input_file, output_file, scale_factor=2.0):
    """Process the SDF file and scale sizes and poses."""
    # Parse the XML file
    tree = ET.parse(input_file)
    root = tree.getroot()

    # Scale model pose
    model = root.find('model')
    if model is not None:
        pose = model.get('pose')
        if pose:
            model.set('pose', scale_pose(pose, scale_factor))

    # Process all links
    for link in root.findall('.//link'):
        # Scale link pose
        pose = link.get('pose')
        if pose:
            link.set('pose', scale_pose(pose, scale_factor))

        # Scale collision and visual elements
        for element in link.findall('.//collision') + link.findall('.//visual'):
            # Scale element pose
            pose = element.get('pose')
            if pose:
                element.set('pose', scale_pose(pose, scale_factor))

            # Scale size in box geometry
            box = element.find('.//box/size')
            if box is not None and box.text:
                box.text = scale_size(box.text, scale_factor)

    # Write the modified XML to the output file
    tree.write(output_file, encoding='utf-8', xml_declaration=True)

def process_world_file(world_file, scale_factor=2.0):
    """Process the world file and scale the gedung_p model."""
    # Parse the world file
    tree = ET.parse(world_file)
    root = tree.getroot()

    # Find the gedung_p model
    for model in root.findall('.//model'):
        if model.get('name') == 'gedung_p':
            # Scale the model's pose
            pose = model.get('pose')
            if pose:
                model.set('pose', scale_pose(pose, scale_factor))
            
            # Scale the model's scale
            scale = model.get('scale')
            if scale:
                model.set('scale', scale_size(scale, scale_factor))
            else:
                model.set('scale', f"{scale_factor} {scale_factor} {scale_factor}")

    # Write the modified world file
    tree.write(world_file, encoding='utf-8', xml_declaration=True)

if __name__ == '__main__':
    world_file = '../../worlds/gedung_p.world'
    scale_factor = 2.0

    try:
        process_world_file(world_file, scale_factor)
        print(f"Successfully scaled gedung_p model in world file by factor {scale_factor}")
    except Exception as e:
        print(f"Error processing world file: {e}") 