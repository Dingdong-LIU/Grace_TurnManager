from flask import Flask, request, jsonify
import torch
from transformers import T5Tokenizer, AutoModelForSeq2SeqLM, GenerationConfig

app = Flask(__name__)

tokenizer = T5Tokenizer.from_pretrained(
    "yuyijiong/T5-large-sentiment-analysis-Chinese-MultiTask"
)
model = AutoModelForSeq2SeqLM.from_pretrained(
    "yuyijiong/T5-large-sentiment-analysis-Chinese-MultiTask"
)
generation_config = GenerationConfig.from_pretrained(
    "yuyijiong/T5-large-sentiment-analysis-Chinese-MultiTask"
)


@app.route("/predict", methods=["POST"])
def predict():
    data = request.get_json()
    input_text = f"判断以下对问题回答的情感极性: {data['conversation']}"
    result = "non-negative"
    # Perform inference using your T5 model
    input_ids = tokenizer(input_text, return_tensors="pt").input_ids
    with torch.no_grad():
        output = model.generate(input_ids, generation_config=generation_config)
    output_str = tokenizer.batch_decode(output, skip_special_tokens=True)
    if '消极' in output_str:
        result = "negative"
    return jsonify({"output": result})


if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0", port=12344)
