from pymongo import MongoClient
import h5py

def save_hdf5_to_mongo(hdf5_file):
    # Match the docker-compose credentials
    client = MongoClient("mongodb://localhost:27017/")

    db = client["robot_data"]       # custom DB
    collection = db["trajectories"]

    with h5py.File(hdf5_file, "r") as f:
        # Insert metadata only (not entire arrays for now)
        collection.insert_one({
            "filename": hdf5_file,
            "groups": list(f.keys())
        })

    print(f"✅ Exported {hdf5_file} to MongoDB")