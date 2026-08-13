from x2coco import voc_get_label_anno, voc_xmls_to_cocojson, spilt_voc_dataset
import os




# 数据集转换
def data_voc_to_coco(voc_anno_dir):
    # voc_anno_dir = 'car_data_coco'
    list_process = ['train', 'valid', 'test']
    # voc 数据结尾符号
    suffixe_voc = '.txt'
    suffixe_coco = '.json'

    voc_anno_list = 'test.txt'
    voc_out_name = 'test.json'

    voc_label_list = 'label_list.txt'
    voc_label_list = os.path.join(voc_anno_dir, voc_label_list)

    voc_anno_path = os.path.join(voc_anno_dir, voc_anno_list)

    output_dir = '.'
    for datas_name in list_process:
        voc_anno_list = os.path.join(voc_anno_dir, datas_name + suffixe_voc)
        voc_out_name = os.path.join(voc_anno_dir, datas_name + suffixe_coco)

        
        assert voc_anno_dir and voc_anno_list and voc_label_list
        label2id, ann_paths = voc_get_label_anno(
            voc_anno_dir, voc_anno_list, voc_label_list)
        # print(ann_paths)
        voc_xmls_to_cocojson(
            annotation_paths=ann_paths,
            label2id=label2id,
            output_dir=output_dir,
            output_file=voc_out_name)

def yaml_cfg_save(voc_anno_dir):
    '''
    num_classes: 6
    metric: COCO
    map_type: integral

    TrainDataset:
    !COCODataSet
        image_dir: images
        anno_path: train.json
        dataset_dir: /home/aistudio/data/car_data_coco
        data_fields: ['image', 'gt_bbox', 'gt_class', 'is_crowd']

    EvalDataset:
    !COCODataSet
        image_dir: images
        anno_path: valid.json
        dataset_dir: /home/aistudio/data/car_data_coco

    TestDataset:
    !ImageFolder
        anno_path: test.json
        dataset_dir: /home/aistudio/data/car_data_coco
    '''
    voc_label_list = 'label_list.txt'
    voc_label_list = os.path.join(voc_anno_dir, voc_label_list)
    label_list = open(voc_label_list, 'r').read().splitlines()
    dataset_dir = '/home/aistudio/data/' + voc_anno_dir
    yml_cfg = {
        'num_classes': len(label_list),
        'metric': 'COCO',
        'map_type': 'integral',
        'TrainDataset': {
            'image_dir': 'images',
            'anno_path': 'train.json',
            'dataset_dir': dataset_dir,
            'data_fields': ['image', 'gt_bbox', 'gt_class', 'is_crowd']
        },
        'EvalDataset': {
            'image_dir': 'images',
            'anno_path': 'valid.json',
            'dataset_dir': dataset_dir,
        }        ,
        'TestDataset': {
            'anno_path': 'test.json',
            'dataset_dir': dataset_dir,
        }
    }

    # 保存为yml文件
    import yaml
    yaml_cfg_path = os.path.join(voc_anno_dir, 'data.yml')
    with open(yaml_cfg_path, 'w', encoding='utf-8') as f:
        yaml.dump(yml_cfg, f, allow_unicode=True, sort_keys=False, default_style='')

    print('yaml_cfg_path:', yaml_cfg_path)
        
    # print(yml_cfg)


if __name__ == '__main__':
    # 分割数据并生成label_list
    test_percent = 0.2
    image_set_dir = 'dataset'
    label_list = spilt_voc_dataset(image_set_dir, test_percent)
    # 生成配置文件
    yaml_cfg_save(image_set_dir)
    
    # 转换voc数据集为coco数据集
    data_voc_to_coco(image_set_dir)
