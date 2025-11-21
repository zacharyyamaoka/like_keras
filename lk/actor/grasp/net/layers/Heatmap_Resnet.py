import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.models as tm

from .DUC import DUC


class HeatmapResNet(nn.Module):
    """
    Torchvision ResNet backbone with:
        PixelShuffle(2) -> DUC(x2) -> DUC(x2) -> Conv2d(3x3) => logits
    """

    def __init__(
        self,
        num_layers: int = 50,  # 18, 34, 50, 101, 152
        out_channels: int = 360,  # heatmap depth
        output_downsample: int = 4,  # {1,4} for this head layout
        upsample_logits: bool = True,
        norm_layer=nn.BatchNorm2d,
        weights: tm.WeightsEnum | None = None,
        input_config="7x7-no-pool",
    ):
        super().__init__()
        assert output_downsample in (1, 4), "output_downsample must be in {1,4}"

        # Unified map of constructors and default weights
        model_map = {
            18: (tm.resnet18, tm.ResNet18_Weights.DEFAULT),
            34: (tm.resnet34, tm.ResNet34_Weights.DEFAULT),
            50: (tm.resnet50, tm.ResNet50_Weights.DEFAULT),
            101: (tm.resnet101, tm.ResNet101_Weights.DEFAULT),
            152: (tm.resnet152, tm.ResNet152_Weights.DEFAULT),
        }
        assert (
            num_layers in model_map
        ), f"num_layers must be one of {list(model_map.keys())}"
        backbone_ctor, default_weights = model_map[num_layers]

        if weights is None:
            weights = default_weights

        # Build backbone with or without dilation
        if output_downsample == 1:
            self._final_stride = 8
            self.backbone = backbone_ctor(
                weights=weights, replace_stride_with_dilation=[False, True, True]
            )
        else:
            self._final_stride = 32
            self.backbone = backbone_ctor(weights=weights)

        # Some different options here for input

        if (
            input_config == "7x7-no-pool"
        ):  # Option A: Keep the 7×7 stem, just remove downsampling
            self.backbone.maxpool = nn.Identity()  # remove /2
            old_conv1 = self.backbone.conv1
            self.backbone.conv1 = nn.Conv2d(
                3,
                old_conv1.out_channels,
                kernel_size=7,
                stride=1,
                padding=3,
                bias=False,
            )
            with torch.no_grad():  # copy weights (same shape) so pretrained weights still work
                self.backbone.conv1.weight.copy_(old_conv1.weight)

        elif (
            input_config == "3x3-no-pool"
        ):  # Option B: Use a CIFAR-style 3×3 stem (common for tiny images)
            self.backbone.maxpool = nn.Identity()  # remove /2
            old_conv1 = self.backbone.conv1  # [64,3,7,7] usually
            self.backbone.conv1 = nn.Conv2d(
                3,
                old_conv1.out_channels,
                kernel_size=3,
                stride=1,
                padding=1,
                bias=False,
            )
            # If using pretrained weights, map 7x7 -> 3x3 by taking the center patch
            if old_conv1.weight.shape[-1] == 7:
                with torch.no_grad():
                    self.backbone.conv1.weight.copy_(old_conv1.weight[:, :, 2:5, 2:5])
        else:
            pass

        self.backbone.fc = nn.Identity()

        # Backbone output channels
        self._backbone_out_ch = 512 if num_layers in (18, 34) else 2048

        # ---- Head ----
        ps1_out_ch = self._backbone_out_ch // 4
        self.shuffle1 = nn.PixelShuffle(2)
        self.duc1 = DUC(
            ps1_out_ch, ps1_out_ch * 4, upscale_factor=2, norm_layer=norm_layer
        )
        self.duc2 = DUC(
            ps1_out_ch, ps1_out_ch * 4, upscale_factor=2, norm_layer=norm_layer
        )
        self.conv_out = nn.Conv2d(ps1_out_ch, out_channels, kernel_size=3, padding=1)

        nn.init.normal_(self.conv_out.weight, std=0.001)
        if self.conv_out.bias is not None:
            nn.init.constant_(self.conv_out.bias, 0)

        self.conv_dim = ps1_out_ch
        self.output_downsample = output_downsample
        self.upsample_logits = upsample_logits

    def _forward_backbone(self, x: torch.Tensor) -> torch.Tensor:
        x = self.backbone.conv1(x)
        x = self.backbone.bn1(x)
        x = self.backbone.relu(x)
        x = self.backbone.maxpool(x)
        x = self.backbone.layer1(x)
        x = self.backbone.layer2(x)
        x = self.backbone.layer3(x)
        x = self.backbone.layer4(x)
        return x

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        feat = self._forward_backbone(x)
        y = self.shuffle1(feat)
        y = self.duc1(y)
        y = self.duc2(y)
        logits = self.conv_out(y)
        if self.upsample_logits:
            logits = F.interpolate(
                logits, size=x.shape[-2:], mode="bilinear", align_corners=False
            )
        return logits
