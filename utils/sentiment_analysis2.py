import time
from langchain_core.prompts import PromptTemplate
from langchain_openai import ChatOpenAI
from langchain_core.output_parsers import StrOutputParser
import os
from flask import Flask, jsonify, request

str_parser = StrOutputParser()
prompt = PromptTemplate(
    template="请判断以下AI和人的对话中人的情感极性: {data}\n 只输出`中性`、`积极`或者`消极`中的一个，不要输出多余字词",
    input_variables=["data"],
)
langchain_llm = ChatOpenAI(
    base_url="http://localhost:8000/v1",
    api_key="No Need",
    model="Qwen/Qwen2-7B-Instruct-GPTQ-Int8",
    verbose=True,
)

chain = prompt | langchain_llm | str_parser

app = Flask(__name__)

os.environ["CUDA_VISIBLE_DEVICES"] = "1"

@app.route("/predict", methods=["POST"])
def predict():
    # record response delay
    start_time = time.time()

    data = request.get_json()
    input_text = data['conversation']
    result = "neutral"
    # Perform inference using your T5 model
    output_str = chain.invoke(
        {
            "data": input_text
        }
    )
    if "消极" in output_str:
        result = "negative"
    elif "中性" in output_str:
        result = "neutral"
    elif "积极" in output_str:
        result = "positive"
    # log the input and output to console
    end_time = time.time()
    print(
        f"Input: {input_text}\n Output: {output_str} \n Execution time: {end_time - start_time} seconds"
    )

    return jsonify({"output": result})


if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0", port=12344)