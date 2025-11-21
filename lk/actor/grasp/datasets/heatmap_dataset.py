from torch.utils.data import Dataset
from typing import Callable
import torch


class HeatmapDataset(Dataset):
    def __init__(
        self,
        input_label_pairs: list[tuple[torch.Tensor, torch.Tensor]],
        transform: Callable | None = None,
        label_transforms: Callable | None = None,
    ):

        self.data = input_label_pairs
        self.transform = transform
        self.label_transforms = label_transforms

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        img, label = self.data[idx]

        if self.transform:
            img = self.transform(img)
        if self.label_transforms:
            label = self.label_transforms(label)

        return img, label
