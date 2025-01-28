import torch
import verification.object_tracking_node.constants as constants
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import math
device = "cuda" if torch.cuda.is_available() else "cpu"

def iou_width_height(boxes1, boxes2):
    intersection = torch.min(boxes1[..., 0], boxes2[..., 0]) * torch.min(
        boxes1[..., 1], boxes2[..., 1]
    )
    union = (
        boxes1[..., 0] * boxes1[..., 1] + boxes2[..., 0] * boxes2[..., 1] - intersection
    )
    return intersection / union

def intersection_over_union(boxes_preds, boxes_labels, box_format="midpoint"):

    if box_format == "midpoint":
        box1_x1 = boxes_preds[..., 0:1] - boxes_preds[..., 2:3] / 2
        box1_y1 = boxes_preds[..., 1:2] - boxes_preds[..., 3:4] / 2
        box1_x2 = boxes_preds[..., 0:1] + boxes_preds[..., 2:3] / 2
        box1_y2 = boxes_preds[..., 1:2] + boxes_preds[..., 3:4] / 2
        box2_x1 = boxes_labels[..., 0:1] - boxes_labels[..., 2:3] / 2
        box2_y1 = boxes_labels[..., 1:2] - boxes_labels[..., 3:4] / 2
        box2_x2 = boxes_labels[..., 0:1] + boxes_labels[..., 2:3] / 2
        box2_y2 = boxes_labels[..., 1:2] + boxes_labels[..., 3:4] / 2

    if box_format == "corners":
        box1_x1 = boxes_preds[..., 0:1]
        box1_y1 = boxes_preds[..., 1:2]
        box1_x2 = boxes_preds[..., 2:3]
        box1_y2 = boxes_preds[..., 3:4]
        box2_x1 = boxes_labels[..., 0:1]
        box2_y1 = boxes_labels[..., 1:2]
        box2_x2 = boxes_labels[..., 2:3]
        box2_y2 = boxes_labels[..., 3:4]

    x1 = torch.max(box1_x1, box2_x1)
    y1 = torch.max(box1_y1, box2_y1)
    x2 = torch.min(box1_x2, box2_x2)
    y2 = torch.min(box1_y2, box2_y2)

    intersection = (x2 - x1).clamp(0) * (y2 - y1).clamp(0)
    box1_area = abs((box1_x2 - box1_x1) * (box1_y2 - box1_y1))
    box2_area = abs((box2_x2 - box2_x1) * (box2_y2 - box2_y1))

    return intersection / (box1_area + box2_area - intersection + 1e-6)


def non_max_suppression(bboxes, iou_threshold, threshold, box_format="midpoint"):
    assert type(bboxes) == list

    bboxes = [box for box in bboxes if box[0] > threshold]
    bboxes = sorted(bboxes, key=lambda x: x[0], reverse=True)[0:50]
    bboxes_after_nms = []

    while bboxes:
        chosen_box = bboxes.pop(0)

        bboxes = [
            box
            for box in bboxes
            if box[5] != chosen_box[5]
            or intersection_over_union(
                torch.tensor(chosen_box[1:5]),
                torch.tensor(box[1:5]),
                box_format=box_format,
            )
            < iou_threshold
        ]

        bboxes_after_nms.append(chosen_box)

    return bboxes_after_nms


def get_bounding_boxes_for_prediction(prediction_tensor, batch_idx=0):
    first_split = prediction_tensor[0][batch_idx]
    second_split = prediction_tensor[1][batch_idx]
    anchors = torch.tensor(constants.anchor_boxes).to(device)
    bounding_boxes = cells_to_bboxes(first_split, anchors[0], constants.split_sizes[0]) + cells_to_bboxes(second_split, anchors[1], constants.split_sizes[1])
    return non_max_suppression(bounding_boxes,iou_threshold=0.01,threshold=0.6) # was 0.8 in training

def cells_to_bboxes(predictions, anchors, split_size):
    num_anchors = len(anchors)
    anchors = anchors.reshape(1, len(anchors), 1, 1, 2) * split_size
    box_predictions = predictions[..., 1:5]


    box_predictions[..., 0:2] = torch.sigmoid(box_predictions[..., 0:2])
    box_predictions[..., 2:] = torch.exp(box_predictions[..., 2:]) * anchors

    scores = torch.sigmoid(predictions[..., 0:1])
    best_class = torch.argmax(predictions[..., 5:], dim=-1).unsqueeze(-1)

    cell_indices = (
        torch.arange(split_size)
        .repeat(3, split_size, 1)
        .unsqueeze(-1)
        .to(predictions.device)
    )
    x = 1 / split_size * (box_predictions[..., 0:1] + cell_indices)
    y = 1 / split_size * (box_predictions[..., 1:2] + cell_indices.permute(0, 2, 1, 3))
    w_h = 1 / split_size * box_predictions[..., 2:4]
    converted_bboxes = torch.cat((scores, x, y, w_h, best_class), dim=-1).reshape(num_anchors * split_size * split_size, 6)
    return converted_bboxes.tolist()