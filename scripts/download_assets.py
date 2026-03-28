import urllib.request
import zipfile
import io
import os

url = "https://github.com/TheRobotStudio/SO-ARM100/archive/refs/heads/main.zip"
print(f"Downloading repository zip from {url}...")
try:
    response = urllib.request.urlopen(url)
    print("Extracting assets folder...")
    with zipfile.ZipFile(io.BytesIO(response.read())) as z:
        for file_info in z.infolist():
            if "Simulation/SO100/assets/" in file_info.filename and not file_info.filename.endswith('/'):
                # Extract to local assets folder
                local_path = os.path.join("assets", os.path.basename(file_info.filename))
                os.makedirs("assets", exist_ok=True)
                with open(local_path, "wb") as f:
                    f.write(z.read(file_info.filename))
                    print(f"Extracted {local_path}")
    print("Asset extraction complete.")
except Exception as e:
    print(f"Error downloading or extracting assets: {e}")
