from pymongo import MongoClient
import numpy as np
import bson

def save_npz_to_mongo(npz_file):
    # Match docker-compose credentials
    client = MongoClient("mongodb://localhost:27017/")
    
    db = client["robot_data"]       # custom DB
    collection = db["trajectories"] # collection for trajectories

    # Load the NumPy data
    data = np.load(npz_file)

    # Convert arrays to lists (MongoDB doesn’t store raw numpy arrays well)
    doc = {
        "filename": npz_file,
        "arrays": {key: data[key].tolist() for key in data.files},
    }

    collection.insert_one(doc)
    print(f"✅ Exported {npz_file} to MongoDB with keys {list(data.files)}")
