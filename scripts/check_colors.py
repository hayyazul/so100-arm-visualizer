import yourdfpy
urdf_model = yourdfpy.URDF.load("so100.urdf")
for link in urdf_model.robot.links:
    for visual in link.visuals:
        print(f"Link {link.name}")
        if visual.material and visual.material.color:
            print(f"  Material: {visual.material.name}, Color: {visual.material.color.rgba}")
        else:
            print(f"  No color specified in URDF visual.")

print("--- Checking Scene Geometries ---")
for name, geom in urdf_model.scene.geometry.items():
    color = geom.visual.material.main_color
    print(f"Geom {name}: main_color={color}")
