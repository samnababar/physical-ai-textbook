---
sidebar_position: 2
---

# Week 2: VLA Architectures

Vision-Language-Action models come in various architectures, but they generally share a common structure: an encoder-decoder framework, often based on the Transformer architecture that has revolutionized natural language processing.

## The Transformer Backbone

At the heart of most modern VLAs is the **Transformer**. The key innovation of the Transformer is the *attention mechanism*, which allows the model to weigh the importance of different parts of the input sequence. In a VLA, this means the model can pay "attention" to specific words in the command and specific regions in the image.

## A General VLA Architecture

A typical VLA can be broken down into the following components:

1.  **Vision Encoder:** This is a neural network (often a pre-trained one like a Vision Transformer or ViT) that takes an image as input and converts it into a sequence of numerical representations, or *tokens*.

2.  **Language Encoder:** This is another neural network (often a pre-trained one like BERT or GPT) that takes the natural language command as input and converts it into a sequence of tokens.

3.  **Multimodal Fusion:** The vision and language tokens are combined. A common technique is to simply concatenate them into a single sequence.

4.  **Action Decoder:** This is a Transformer-based decoder that takes the fused vision and language tokens as input and generates a sequence of action tokens. These action tokens represent the robot's motor commands. The decoder is *autoregressive*, meaning it generates one action token at a time, and each new prediction is based on the previous ones.

### Conceptual Code for a VLA

```python
import torch
from torch import nn

# These are placeholder modules
class VisionEncoder(nn.Module):
    # ...
    def forward(self, image):
        # image -> vision_tokens
        return vision_tokens

class LanguageEncoder(nn.Module):
    # ...
    def forward(self, text):
        # text -> language_tokens
        return language_tokens

class ActionDecoder(nn.Module):
    # ...
    def forward(self, fused_tokens):
        # fused_tokens -> action_tokens
        return action_tokens

class VLA(nn.Module):
    def __init__(self):
        super().__init__()
        self.vision_encoder = VisionEncoder()
        self.language_encoder = LanguageEncoder()
        self.action_decoder = ActionDecoder()

    def forward(self, image, text):
        vision_tokens = self.vision_encoder(image)
        language_tokens = self.language_encoder(text)

        # 3. Multimodal Fusion
        fused_tokens = torch.cat([vision_tokens, language_tokens], dim=1)

        # 4. Action Decoding
        action_tokens = self.action_decoder(fused_tokens)
        return action_tokens
```

![RT-2: A VLA from Google DeepMind](https://storage.googleapis.com/deepmind-media/DeepMind.com/Blog/rt-2-new-model-translates-vision-and-language-into-action/RT-2-product-image_16_9.jpg)