import re

def clean_address(address_string: str) -> str:
    words = ["kel", "kec", "kelurahan", "kecamatan", "indonesia", "jl", "jalan", 
             "kota", "samping", "rumah", "sakit", "rs", "no", "nomor", " rt", "rw", "idn", "blok"]
    new_string = address_string.lower()
    new_string = re.sub(r'[^a-z]+', ' ', new_string)
    for word in words:
        new_string = new_string.replace(word + " ", ' ')
        new_string = new_string.replace(" " + word, ' ')
    new_string = new_string + " indonesia"
    new_string = re.sub(' +', ' ', new_string)
    split_word = new_string.split()
    new_string = " ".join(sorted(set(split_word), key=split_word.index))
    return new_string

def validate_lat_lon(coord_str: str) -> bool:
    x = re.search(r"^(-?\d+(\.\d+)?),\s*(-?\d+(\.\d+)?)$", coord_str)
    return bool(x)

def create_demand_model(data: dict, demand: str) -> list:
    task_list = data.get('task_list', [])
    arr_demand = []
    for task in task_list:
        arr_demand.append(task.get(demand, 0))
    return arr_demand
