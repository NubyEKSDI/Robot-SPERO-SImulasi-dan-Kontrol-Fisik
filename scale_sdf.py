#!/usr/bin/env python3
import re
import sys
import shutil
from pathlib import Path

def scale_number(match):
    """Scale a number by 2x"""
    num = float(match.group(0))
    return str(num * 2)

def scale_sdf_file(input_file, output_file):
    """Scale all numerical values in an SDF file by 2x"""
    with open(input_file, 'r') as f:
        content = f.read()
    
    # Scale numbers in size tags
    content = re.sub(r'<size>([\d.]+)\s+([\d.]+)\s+([\d.]+)</size>',
                    lambda m: f'<size>{float(m.group(1))*2} {float(m.group(2))*2} {float(m.group(3))*2}</size>',
                    content)
    
    # Scale numbers in pose tags
    content = re.sub(r'<pose[^>]*>([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)</pose>',
                    lambda m: f'<pose frame="">{float(m.group(1))*2} {float(m.group(2))*2} {float(m.group(3))*2} {float(m.group(4))*2} {float(m.group(5))*2} {float(m.group(6))*2}</pose>',
                    content)
    
    # Write the scaled content to the output file
    with open(output_file, 'w') as f:
        f.write(content)

if __name__ == "__main__":
    input_file = "src/bcr_bot/models/gedung_p/model.sdf"
    output_file = "src/bcr_bot/models/gedung_p/model_scaled.sdf"
    
    # Create backup of original file
    backup_file = input_file + ".backup"
    shutil.copy2(input_file, backup_file)
    
    # Scale the file
    scale_sdf_file(input_file, output_file)
    print(f"Original file backed up to: {backup_file}")
    print(f"Scaled file saved to: {output_file}") 