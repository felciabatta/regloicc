"""Extract & log data streams for live experiments."""

import csv
import gc
import time
from datetime import datetime, timedelta

from keras import backend as K
from moma import MoMaAnalysis
from utils import get_img_name, get_next_log_filename


class CSVLogger:
    """Write structured records to a CSV file."""

    def __init__(self, path, fieldnames):
        self.path = path
        self.fieldnames = fieldnames

        self.file = None
        self.writer: csv.DictWriter | None = None

    def __enter__(self):
        self.file = open(self.path, "x", newline="")

        self.writer = csv.DictWriter(self.file, fieldnames=self.fieldnames)

        self.writer.writeheader()
        self.file.flush()

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    @property
    def writer_active(self):
        return self.writer is not None

    @property
    def n_fields(self):
        return len(self.fieldnames)

    def write(self, *args, **kwargs):
        record = dict() if not args else args[0]
        record |= kwargs
        if len(record) == self.n_fields and self.writer_active:
            return self.writer.writerow(record)
        else:
            raise ValueError(
                f"Incorrect number of fields supplied ({len(record)})"
            )

    def flush(self):
        if self.file is not None:
            return self.file.flush()

    def close(self):
        if self.file is not None:
            self.file.close()
            self.file = None
            self.writer = None


def log_fluorescence(
    dir_parent, id_pos, channels, moma: MoMaAnalysis, t_duration, t_step
):
    """Monitor every `t_step` for `t_duration` mins.

    Writes fluorescence values to log file.
    """

    T_END = datetime.now() + timedelta(minutes=t_duration)
    print(f" Monitoring {id_pos} until {T_END.strftime('%H:%M:%S')}")
    frames_processed = set()
    is_first = True
    id_frame = 0

    while datetime.now() < T_END:
        # FILENAMES FOR CURRENT id_frame (multi-channel)
        t_now = datetime.now().strftime("%H:%M:%S")
        img_names = [get_img_name(id_pos, id_frame, chan) for chan in channels]
        print(dir_parent + img_names[0])
        print(f"[{id_pos} | {t_now}] Checking for {img_names[0]}...")

        # CHECK IF IMAGE IS PRESENT FOR CURRENT id_frame
        found_img, imgs = moma.read_imgs(img_names)
        if found_img and id_frame not in frames_processed:
            print(f"{id_pos}: FOUND {img_names}")

            if is_first and imgs is not None:  # check/set reference image
                moma.set_reference(imgs[0])

            # LOAD FRAME IMAGES
            moma.load_frame(imgs)

            # SEGMENT TARGET BRANCH
            moma.segment_branches(id_frame, is_first, branch_target=4)
            is_first = False

            # EVALUATE FLUORESCENCE IN TARGET BRANCH
            fromcell = False
            moma.fluo_from_branch(id_frame)
            fluo_val = moma.extracted_features_channel[id_frame]

            # SEGMENT CELLS IN TARGET BRANCH
            segment = moma.segment_cells()

            # EVALUATE FLUORESCENCE FROM CELL
            if segment:
                moma.fluo_from_cell(id_frame)
                fluo_val = moma.extracted_features_cell[id_frame]
                fromcell = True

            # WRITE TO FILE
            write_feature_to_file(None, id_pos, id_frame, fluo_val, fromcell)
            frames_processed.add(id_frame)
            id_frame += 1

            # BACKEND CLEANUP
            K.clear_session()
            gc.collect()

            # WAIT t_step MINUTES BEFORE NEXT CHECK
            time.sleep(t_step * 60)
        else:
            # TRY AGAIN EVERY 1 SEC IF NOT FOUND
            # - Useful in case of syncing issues
            print(f"{id_pos}: NOT FOUND {img_names}. Retrying...")
            retry_time = datetime.now() + timedelta(seconds=1)
            while not found_img and datetime.now() < retry_time:
                time.sleep(1)
                found_img, _ = moma.read_imgs(img_names)

    # END MONITORING WITH MESSAGE
    t_now = datetime.now().strftime("%H:%M:%S")
    print(f"Monitoring ended for {id_pos} at {t_now}")
    pass


def write_feature_to_file(filepath, id_pos, id_frame, feature_value, fromcell):
    """write_feature_to_file

    :param filepath: Description
    :param id_pos: Description
    :param id_frame: Description
    :param feature_value: Description
    :param fromcell: Description

    CSV Format
    ---------
    `"Y-m-d H:M:S", int, int, int, bool, str`

    """

    if filepath is None:
        filepath = get_next_log_filename()
    with open(filepath, "a", newline="") as f:
        writer = csv.writer(f)
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        writer.writerow(
            [now, id_pos, id_frame, feature_value, fromcell, "UPDATED"]
        )


def main():
    """Example usage"""

    # filepath = get_next_log_filename()
    # Paths "~/Exp_1105/2025_07_12_12_53_09--210325/Mark_and_Find 001"
    # "~/2025_08_12_11_39_25--210325/Mark_and_Find 001/"
    # "/mnt/y/Sara/Exp_1105/2025_11_11_12_57_32--210325/Mark_and_Find 001/"

    # CONSTANTS
    DIR_PARENT = "~/Exp_1105/2025_11_20_14_17_11--210325/Mark_and_Find 001/"
    ID_POS = 1  # "Position001"#--t00--C.tif"
    CHANNELS = [0, 1, 2]  # phase, fluo1, fluo2
    T_DURATION = 24 * 60  # min
    T_STEP = 10  # min

    # SETUP ANALYSIS
    moma = MoMaAnalysis(
        dir_parent=DIR_PARENT,
        name_pos="Position001--t",
        path_roi="~/MM-imgs/assets/model/unet_momachambers_seg.hdf5",
        path_seg="~/MM-imgs/assets/model/unet_moma_seg.hdf5",
    )

    # START LOGGING
    log_fluorescence(DIR_PARENT, ID_POS, CHANNELS, moma, T_DURATION, T_STEP)
    pass


if __name__ == "__main__":
    main()
