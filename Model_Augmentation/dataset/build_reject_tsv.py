import os
import random
import pandas as pd

from tqdm import tqdm


# DST_TSV_FOLDER = 'FELICE_demo3/rejects'
# DST_TSV_FOLDER = 'FELICE_demo3/rejects'
DST_TSV_FOLDER = "full_dataset_v1/rejects"


for language in ['eng', 'ita']:
    # build the path
    dst_lan_path = os.path.join(DST_TSV_FOLDER, language)
    dst_dataset_path = os.path.join(dst_lan_path, 'clips')

    # load sample name
    file_list = os.listdir(dst_dataset_path)
    random.shuffle(file_list)

    # build the tsv
    dataset_iter = tqdm(file_list)
    rows = list()
    for file in dataset_iter:
        rows.append([file.split('_')[0], file, None, None, None, None, None, language, None])

        dataset_iter.set_description('Building {} tsv file'.format(language.upper()))
    
    # build Dataframe
    src_tsv_data = pd.DataFrame(rows, columns=['client_id', 'path', 'sentence', 'up_votes', 'age', 'gender', 'accent', 'locale', 'segment'])

    # save the tsv file
    dst_tsv_file = os.path.join(dst_lan_path, 'train.tsv')
    src_tsv_data.to_csv(dst_tsv_file, sep='\t')