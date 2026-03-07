import xml.etree.ElementTree as ET

def strip_visuals_from_urdf(input_path: str, output_path: str) -> None:
    tree = ET.parse(input_path)
    root = tree.getroot()

    for link in root.findall('link'):
        for visual in link.findall('visual'):
            link.remove(visual)
        for collision in link.findall('collision'):
            link.remove(collision)

    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    print(f"Simplified URDF saved to {output_path}")

if __name__ == "__main__":
    strip_visuals_from_urdf('so100.urdf', 'so100_kinematics.urdf')
