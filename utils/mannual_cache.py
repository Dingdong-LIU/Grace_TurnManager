import json

def read_cantonese_translation_json(path= "data/cantonese_translation.json"):
    with open(path, 'r', encoding='utf-8') as f:
        file_content = json.load(f)
    cantonese_sentence_with_annotation = list(file_content.values())
    print(cantonese_sentence_with_annotation)
    return cantonese_sentence_with_annotation

if __name__ == "__main__":
    read_cantonese_translation_json()