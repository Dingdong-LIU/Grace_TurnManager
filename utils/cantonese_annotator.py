import json
from thefuzz import fuzz, process

class Cantonese_Annotator:
    def __init__(self, file_dir="/home/grace_team/HKUST_GRACE/Grace_Project/Grace_TurnManager/data/cantonese_translation.json") -> None:
        self.cantonese_database_file = file_dir
        self.annotator = self.__init_database()
    
    def __init_database(self) -> dict:
        with open(self.cantonese_database_file, "r", encoding="utf-8") as f:
            database = json.load(f)
        return database

    def lookup_annotation(self, text) -> str:
        return self.annotator.get(text, text)

    def fuzz_lookup_annotation(self, text) -> str:
        look_up_result = ""
        hash_lookup = self.annotator.get(text, None)
        if hash_lookup:
            look_up_result = hash_lookup
        else:
            # Do fuzz match
            candidate_sentences = self.annotator.keys()
            fuzz_key = process.extractOne(text, candidate_sentences)
            if fuzz_key and fuzz_key[1] > 90:
                fuzz_lookup = self.annotator.get(fuzz_key[0], text)
                look_up_result = fuzz_lookup
            else:
                look_up_result = text
        return look_up_result