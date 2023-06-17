#import torch
from detectron2.utils.logger import setup_logger

setup_logger()

# import some common detectron2 utilities
from detectron2.engine import DefaultPredictor
from detectron2 import model_zoo
from detectron2.config import get_cfg


def load_model():
    cfg = get_cfg()
    model_file = "COCO-Detection/faster_rcnn_R_50_C4_1x.yaml"
    cfg.merge_from_file(model_zoo.get_config_file(model_file))
    cfg.MODEL.WEIGHTS = "best_model.pth"  # path to the model we just trained
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.92  # set a custom testing threshold
    cfg.MODEL.DEVICE = 'cpu'
    # if not torch.cuda.is_available():
    #     cfg.MODEL.DEVICE = 'cpu'
    cfg.MODEL.ROI_HEADS.NUM_CLASSES = 2
    predictor = DefaultPredictor(cfg)

    return predictor


