import torch
import torch.nn as nn
from typing import List, Dict


class MaskedBCEWithLogitsLoss(nn.BCEWithLogitsLoss):
    """
    Check nn.BCEWithLogitsLoss for kwargs defn.

    BCEWithLogits computed ONLY at executed (c,h,w) indices per batch element.

    Forward args:
        logits:        [B, C, H, W]
        executed_list: list length B, each a LongTensor [Ki, 3] with (c,h,w)
        labels_list:   list length B, each a Float/Bool Tensor [Ki] with {0,1}

    Returns:
        loss: Tensor (scalar if reduction != "none")

    ---

    From the tossing bot paper: "We pass gradients only through the single pixel i on which the grasping primitive was executed. All other pixels backpropagate with 0 loss."

    ##### Key ideas to implement:
    - Instead of applying BCE loss over the entire (C, H, W) tensor, select logits at the executed (c, h, w) indices.
    - This ensures the computational graph only flows through those selected entries, so gradients elsewhere are zero.
    - To handle multiple executed actions per sample, gather all relevant indices, compute the loss for each, and average.

    """

    def __init__(self, *args, **kwargs):
        super().__init__(
            *args, **kwargs
        )  # intention is to pass through, cleaner than copy pasting args here
        self.info: Dict[str, torch.Tensor] = {}

    def forward(
        self,
        logits: torch.Tensor,
        executed_list: List[torch.Tensor],
        labels_list: List[torch.Tensor],
    ) -> torch.Tensor:
        """
        logits: [B, C, H, W]
        executed_list: list length B, each a LongTensor [Ki, 3] with (c,h,w)
        labels_list:   list length B, each a Float/Bool Tensor [Ki] with {0,1}
        """
        B, C, H, W = logits.shape
        device = logits.device

        """ 
            Currently lists look something like:

            executed_list = [
                [[3, 20, 15], [7, 40, 10]],    # B0 x 2
                [[7, 40, 10]],                 # B1 x 1
                [],                            # B2 x 0 (no action)
                [[5, 50, 50], [5, 51, 50], [5, 52, 50]],  # B3 x 3
            ]
            labels_list = [
                [1, 0],
                [1],
                [],
                [0, 0, 1],
            ]

            We want to convert into a single list to easily index the logics directly with a single look up.

            b_all = [0, 0, 1, 3, 3, 3] # Notice how batch 2 disappears..
            c_all = [3, 7, 7, 5, 5, 5]
            h_all = [20, 40, 40, 50, 51, 52]
            w_all = [15, 10, 10, 50, 50, 50]
            y_all = [1, 0, 1, 0, 0, 1]

        """
        # Flatten variable-K actions across the batch
        b_all, c_all, h_all, w_all, y_all = [], [], [], [], []
        for i, (inds_i, y_i) in enumerate(zip(executed_list, labels_list)):
            if inds_i is None or len(inds_i) == 0:  # no action executed in this batch
                continue
            inds_i = inds_i.to(device)  # [Ki, 3] (c,h,w)
            y_i = y_i.to(device).float()  # [Ki]    (0/1)
            Ki = inds_i.shape[0]

            assert (
                inds_i.shape[0] == y_i.numel()
            ), f"labels_list[{i}] length must match executed_list[{i}].shape[0]"
            assert (inds_i[:, 0] >= 0).all() and (inds_i[:, 0] < C).all()
            assert (inds_i[:, 1] >= 0).all() and (inds_i[:, 1] < H).all()
            assert (inds_i[:, 2] >= 0).all() and (inds_i[:, 2] < W).all()

            b_all.append(torch.full((Ki,), i, device=device, dtype=torch.long))
            c_all.append(inds_i[:, 0].long())
            h_all.append(inds_i[:, 1].long())
            w_all.append(inds_i[:, 2].long())
            y_all.append(y_i)

        if len(b_all) == 0:  # If no actions executed in the whole batch, return zero
            return logits.new_tensor(0.0, requires_grad=True)

        b_all = torch.cat(b_all)  # [N]
        c_all = torch.cat(c_all)  # [N]
        h_all = torch.cat(h_all)  # [N]
        w_all = torch.cat(w_all)  # [N]
        y_all = torch.cat(y_all)  # [N]

        # Index only the executed logits -> gradients flow only here
        selected_logits = logits[b_all, c_all, h_all, w_all]  # [N]

        # Standard BCEWithLogitsLoss reduction
        loss = super().forward(selected_logits, y_all)

        # Save debug info for later inspection
        self.info = {
            "b_all": b_all.detach(),
            "c_all": c_all.detach(),
            "h_all": h_all.detach(),
            "w_all": w_all.detach(),
            "y_all": y_all.detach(),
        }

        return loss


if __name__ == "__main__":

    # How to make sure that the this is working as expected?

    # 1. Just calculate loss with random logits, and make sure no errors
    print(
        "\n=== TEST 1: Just calculate loss with random logits, and make sure no errors ==="
    )
    B, C, H, W = 4, 12, 64, 64
    logits = torch.randn(B, C, H, W, requires_grad=True)

    executed_list = [
        torch.tensor([[3, 20, 15], [7, 40, 10]]),  # K0=2
        torch.tensor([[7, 40, 10]]),  # K1=1
        torch.tensor([], dtype=torch.long).reshape(0, 3),  # K2=0
        torch.tensor([[5, 50, 50], [5, 51, 50], [5, 52, 50]]),  # K3=3
    ]
    labels_list = [
        torch.tensor([1, 0], dtype=torch.float32),
        torch.tensor([1], dtype=torch.float32),
        torch.tensor([], dtype=torch.float32),
        torch.tensor([0, 0, 1], dtype=torch.float32),
    ]

    b_all = torch.tensor([0, 0, 1, 3, 3, 3])
    c_all = torch.tensor([3, 7, 7, 5, 5, 5])
    h_all = torch.tensor([20, 40, 40, 50, 51, 52])
    w_all = torch.tensor([15, 10, 10, 50, 50, 50])
    y_all = torch.tensor([1, 0, 1, 0, 0, 1])

    loss_fn = MaskedBCEWithLogitsLoss(reduction="mean")
    loss = loss_fn(logits, executed_list, labels_list)
    loss.backward()
    print("loss:", float(loss))
    logits.grad = None

    expected_values = dict()
    expected_values["b_all"] = b_all
    expected_values["c_all"] = c_all
    expected_values["h_all"] = h_all
    expected_values["w_all"] = w_all
    expected_values["y_all"] = y_all

    for key, value in loss_fn.info.items():
        # print(key, value)
        assert torch.all(
            value == expected_values[key]
        ), f"Expected {key} to be {expected_values[key]}, but got {value}"

    # 2. Make logits high and low depending on y label. See sigmoid function.
    print(
        "\n=== TEST 2: Make logits high and low depending on y label. See sigmoid function. ==="
    )

    for i, y in enumerate(y_all):
        zero_loss_logit = 100 if y == 1 else -100
        logits.data[b_all[i], c_all[i], h_all[i], w_all[i]] = zero_loss_logit
    loss = loss_fn(logits, executed_list, labels_list)
    assert abs(float(loss)) < 1e-8, f"Expected loss to be near zero, got {float(loss)}"

    # logits = torch.zeros_like(logits, requires_grad=True)
    # logits = torch.ones_like(logits, requires_grad=True)
    # with torch.no_grad():
    #     logits.fill_(1.0)
    # # Set logits to one or zeros to see that the scaling factors are the same.
    # # If the logits are different values, then the scalling factors will be different beacuse the gradient depends on the specific logit value

    # 3. Make sure gradients are only at executed indices
    print("\n=== TEST 3: Make sure gradients are only at executed indices ===")

    logits = torch.randn(B, C, H, W, requires_grad=True)
    loss = loss_fn(logits, executed_list, labels_list)
    loss.backward()
    print("loss:", float(loss))

    assert logits.grad is not None, "Gradients are None"
    print(f"logits.grad shape: {logits.grad.shape}")

    executed_mask = torch.zeros_like(logits.grad, dtype=torch.bool)
    executed_mask[b_all, c_all, h_all, w_all] = True

    executed_grads = logits.grad[executed_mask]
    print(f"Gradients at executed positions: {executed_grads}")
    print(f"Max gradient at executed positions: {executed_grads.abs().max():.6f}")
    print(f"Min gradient at executed positions: {executed_grads.abs().min():.6f}")
    assert (
        executed_grads.abs().max() > 1e-8
    ), f"Expected executed gradients to be non-zero, but got {executed_grads.abs().max()}"

    # Check gradients at non-executed positions (should be zero)
    non_executed_grads = logits.grad[~executed_mask]
    non_executed_grads_count = (non_executed_grads.abs() > 1e-8).sum()
    print(
        f"Count of non-zero gradients at non-executed positions: {non_executed_grads_count}"
    )
    assert (
        non_executed_grads_count == 0
    ), f"Expected non-executed gradients to be zero, but got {non_executed_grads_count}"

    print("\n=== TEST 4: Check Zero loss on unmasked loss func ===")

    full_loss_fn = nn.BCEWithLogitsLoss(reduction="mean")
    full_labels = torch.zeros_like(logits)
    logits_full = logits.detach().clone().requires_grad_(True)

    with torch.no_grad():
        logits_full.fill_(-100)

    loss_full = full_loss_fn(logits_full, full_labels)
    print("loss_full:", float(loss_full))
    assert (
        abs(float(loss_full)) < 1e-8
    ), f"Expected loss to be near zero, got {float(loss_full)}"

    print(
        "\n=== TEST 5: Check zero loss on unmasked loss function after inserting the label ==="
    )

    with torch.no_grad():
        full_labels[b_all, c_all, h_all, w_all] = torch.tensor(y_all, dtype=torch.float)
        for i, y in enumerate(y_all):
            zero_loss_logit = 100 if y == 1 else -100
            logits_full[b_all[i], c_all[i], h_all[i], w_all[i]] = zero_loss_logit

    loss_full = full_loss_fn(logits_full, full_labels)
    assert (
        abs(float(loss_full)) < 1e-8
    ), f"Expected loss to be near zero, got {float(loss_full)}"

    print(
        "\n=== TEST 6: Check scalling ratio when the predicted logits are all the same value, but the labels are different ==="
    )
    logits_full = logits.detach().clone().requires_grad_(True)

    with torch.no_grad():
        for i, y in enumerate(y_all):
            if i in [0, 1]:
                val = 0.0
            else:
                val = torch.empty(1).uniform_(-2, 2).item()
            logits_full[b_all[i], c_all[i], h_all[i], w_all[i]] = val
            logits[b_all[i], c_all[i], h_all[i], w_all[i]] = val

    logits_full.grad = None
    loss_full = full_loss_fn(logits_full, full_labels)
    loss_full.backward()

    logits.grad = None
    loss = loss_fn(logits, executed_list, labels_list)
    loss.backward()

    full_executed_grads = logits_full.grad[executed_mask]
    executed_grads = logits.grad[executed_mask]

    gradient_ratio = executed_grads / full_executed_grads

    print(f"Full tensor executed gradients: {full_executed_grads}")
    print(f"Masked BCE executed gradients:  {executed_grads}")
    print(f"Ratio of executed gradients: {gradient_ratio}")

    total_positions = B * C * H * W
    executed_positions = len(b_all)
    non_executed_positions = total_positions - executed_positions
    diff_in_scaling_factor = (1 / 6) / (1 / total_positions)

    print(f"Number of positions: {total_positions}")
    print(f"Number of executed positions: {executed_positions}")
    print(f"Number of non-executed positions: {non_executed_positions}")
    print(
        f"Diff in scaling factor (noticed its the same as the ratio above!): {diff_in_scaling_factor:.4f}"
    )

    # Check that each value in the gradient ratio tensor is equal to diff_in_scaling_factor
    assert torch.allclose(
        gradient_ratio,
        torch.full_like(gradient_ratio, diff_in_scaling_factor),
        atol=1e-6,
    ), f"Gradient ratio values do not match expected scaling factor {diff_in_scaling_factor}"

    # logits was the same above! so these should be the same!
    assert torch.allclose(
        full_executed_grads[0].abs(), full_executed_grads[1].abs(), atol=1e-8
    )
    assert torch.allclose(executed_grads[0].abs(), executed_grads[1].abs(), atol=1e-8)

    # Check non-executed gradients in full BCE
    full_non_executed = logits_full.grad[~executed_mask]
    full_non_zero_count = (full_non_executed.abs() > 1e-8).sum()
    print(f"Full BCE non-executed gradient count: {full_non_zero_count}")
    print(f"Full BCE non-executed gradient max: {full_non_executed.abs().max():.10f}")
    assert (
        full_non_zero_count > 0
    ), f"Expected non-executed gradients to be non-zero, but got {full_non_zero_count}"

    print("\nAll tests passed!")
