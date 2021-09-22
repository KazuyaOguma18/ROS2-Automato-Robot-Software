You have to remove all of files without object-detection.pbtxt and ssd_mobilenet_v1_pets.config in training directory before training.

Download model and then extract it
``` bash
wget http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v1_coco_11_06_2017.tar.gz
```

Training
``` bash
python train.py --logtostderr --train_dir=training/ --pipeline_config_path=training/ssd_mobilenet_v1_pets.config
```

Make graph
``` bash
python export_inference_graph.py \
    --input_type image_tensor \
    --pipeline_config_path training/ssd_mobilenet_v1_pets.config \
    --trained_checkpoint_prefix training/model.ckpt-????? \
    --output_directory tomato_graph
```

