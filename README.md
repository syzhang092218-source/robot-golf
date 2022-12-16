# robot-golf
Course project for 6.4212 Robotic Manipulation @ MIT, Fall 2022.

[Let the Golf Fly: Playing golf with a KUKA robot arm!](docs/6.4212%20Report%20-%20Let%20the%20Golf%20Fly.pdf)

Author: [Songyuan Zhang](https://syzhang092218-source.github.io), [Mingxin Yu](https://mingxiny.github.io/)

## Dependencies

To install the requirements:

```bash
conda create -n robot-golf python=3.8
conda activate robot-golf
pip install -r requirements.txt
```

## Installation
```bash
pip install -e .
```

## Run
Use the following command to run the model:
```bash
python main.py
```

We also provide several options for the model. If one want to use linear regression other than supervised learning to fit the dynamics, use
```bash
python main.py --no-learning
```

If one want to train the supervised learning model from scratch, use
```bash
python main.py --train
```

# Results:
Our results are presented on [YouTube](https://youtu.be/MeRavb_BTiU)
