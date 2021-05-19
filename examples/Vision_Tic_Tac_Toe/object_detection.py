import numpy as np
from pyniryo import *


class Object:
    def __init__(self, pose, shape, color, angle, cx_rel, cy_rel, cx, cy):
        self.obj_pos = pose  # pos relative to the arm
        self.shape = shape  # ObjectShape.SQUARE || ObjectShape.CIRCLE
        self.color = color  # Color.<color>
        self.angle = angle  # angle relative to the arm
        self.cx_rel = cx_rel  # pos relative to workspace
        self.cy_rel = cy_rel
        self.cx = cx
        self.cy = cy


class ObjectDetection:

    def __init__(self, client, observation_pose, workspace_name, global_z_offset, grid_size):

        self.__client = client
        self.__img = None
        self.__workspace_img = None

        self.__workspace_name = workspace_name
        self.__grid_size = grid_size
        self.__observation_pose = observation_pose
        self.__global_z_offset = global_z_offset

        self.__img_mask_rgb = []

        self.__color_rgb = [ObjectColor.RED, ObjectColor.GREEN, ObjectColor.BLUE]
        self.__color_hsv = [ColorHSV.RED.value, ColorHSV.GREEN.value, ColorHSV.BLUE.value]

    def detect_all_object(self):
        self.wait_for_workspace()
        objects_list = self.__extract_objects()
        self.__annotate_image_and_display()
        return objects_list

    def wait_for_workspace(self):
        while not self.__extract_workspace():
            print("Unable to get objects in image (check that landmarks are visible)")
            img = resize_img(self.__img, width=600, height=600)
            show_img("robot view", img)

    def __capture_image(self):
        self.__client.move_pose(*self.__observation_pose)
        mtx, dist = self.__client.get_camera_intrinsics()
        img_compressed = self.__client.get_img_compressed()
        img_raw = uncompress_image(img_compressed)
        self.__img = undistort_image(img_raw, mtx, dist)

    def __extract_workspace(self):
        self.__capture_image()
        self.__workspace_img = extract_img_workspace(self.__img, workspace_ratio=1)

        return self.__workspace_img is not None

    def __morpho_transformation_rgb(self):
        self.__img_mask_rgb = []
        img_threshold_rgb = []

        for x in range(0, 3):
            img_threshold_rgb.append(threshold_hsv(self.__workspace_img, *self.__color_hsv[x]))

            img_erode = img_threshold_rgb[x]
            img_erode = morphological_transformations(img_erode, morpho_type=MorphoType.ERODE, kernel_shape=(14, 14),
                                                      kernel_type=KernelType.ELLIPSE)
            img_erode = morphological_transformations(img_erode, morpho_type=MorphoType.DILATE, kernel_shape=(10, 10),
                                                      kernel_type=KernelType.ELLIPSE)

            self.__img_mask_rgb.append(img_erode)

    def __extract_objects(self):
        self.__morpho_transformation_rgb()

        cnts_rgb = []
        objects_list = []

        for x in range(0, 3):
            cnts_rgb.append(biggest_contours_finder(self.__img_mask_rgb[x], 9))

            # extract the object position, angle and color
            if cnts_rgb[x] is not None:
                for cnt in cnts_rgb[x]:
                    cx, cy = get_contour_barycenter(cnt)
                    cx_rel, cy_rel = relative_pos_from_pixels(self.__workspace_img, cx, cy)
                    angle = get_contour_angle(cnt)
                    obj_pose = self.__client.get_target_pose_from_rel(self.__workspace_name, height_offset=0.0,
                                                                      x_rel=cx_rel, y_rel=cy_rel,
                                                                      yaw_rel=angle)
                    objects_list.append(Object(obj_pose, None, self.__color_rgb[x], 0, cx_rel, cy_rel, cx, cy))

        for obj in objects_list:
            obj.obj_pos.z += self.__global_z_offset

        return objects_list

    def __annotate_image_and_display(self):
        # convert gray scale to BGR for debug display
        img_erode_red = cv2.cvtColor(self.__img_mask_rgb[0], cv2.COLOR_GRAY2BGR, 0)
        img_erode_green = cv2.cvtColor(self.__img_mask_rgb[1], cv2.COLOR_GRAY2BGR, 0)
        img_erode_blue = cv2.cvtColor(self.__img_mask_rgb[2], cv2.COLOR_GRAY2BGR, 0)

        a, img = debug_markers(self.__img)

        # resize all imgs
        img = resize_img(img, width=600, height=600)
        img_work = resize_img(self.__workspace_img, width=300, height=300)
        img_erode_red = resize_img(img_erode_red, width=300, height=300)
        img_erode_green = resize_img(img_erode_green, width=300, height=300)
        img_erode_blue = resize_img(img_erode_blue, width=300, height=300)

        self.__draw_debug(img_work, self.__grid_size)
        self.__draw_debug(img_erode_red, self.__grid_size)
        self.__draw_debug(img_erode_green, self.__grid_size)
        self.__draw_debug(img_erode_blue, self.__grid_size)

        blanc = np.zeros((35, 300, 3), np.uint8)

        img_work = concat_imgs((img_work, blanc), 0)
        img_erode_red = concat_imgs((img_erode_red, blanc), 0)
        img_erode_green = concat_imgs((img_erode_green, blanc), 0)
        img_erode_blue = concat_imgs((img_erode_blue, blanc), 0)

        img_work = resize_img(img_work, height=300)
        img_erode_red = resize_img(img_erode_red, height=300)
        img_erode_green = resize_img(img_erode_green, height=300)
        img_erode_blue = resize_img(img_erode_blue, height=300)

        # label all images
        add_annotation_to_image(img, "  " * 40, write_on_top=False)
        add_annotation_to_image(img, " " * 22 + "Camera", write_on_top=False)
        add_annotation_to_image(img_work, "        Workspace         ", write_on_top=False)
        add_annotation_to_image(img_erode_red, "          Red             ", write_on_top=False)
        add_annotation_to_image(img_erode_green, "          Green           ", write_on_top=False)
        add_annotation_to_image(img_erode_blue, "          Blue            ", write_on_top=False)

        # concat all images
        img_l = concat_imgs((img_work, img_erode_red), 1)
        img_r = concat_imgs((img_erode_green, img_erode_blue), 1)

        img_res = concat_imgs((img_l, img_r), 0)
        img_res = concat_imgs((img, img_res), 1)

        show_img("robot view", img_res, wait_ms=100)

    def put_objects_on_grid(self, objects_list):
        grid = []
        # build the grid
        for x in range(0, self.__grid_size[0]):
            grid.append([])
        for x in range(0, self.__grid_size[0]):
            for y in range(0, self.__grid_size[1]):
                grid[x].append([])

        # put all objects in the grid
        for elem in objects_list:
            y = elem.cx_rel * self.__grid_size[0]
            x = elem.cy_rel * self.__grid_size[1]
            grid[int(x)][int(y)].append(elem)

        return grid

    @staticmethod
    def __draw_debug(img, grid_size):
        db_cnt = 3
        sx, sy, *sz = img.shape
        seg = int(sx / grid_size[0])
        off_x = 0

        if grid_size == [4, 4]:
            # green circle stock
            img[0:sx, seg * 0:seg * 0 + db_cnt] = [255, 0, 0]
            img[0:sx, seg * 1 - db_cnt:seg * 1] = [255, 0, 0]
            img[seg * 0:seg * 0 + db_cnt, 0:seg] = [255, 0, 0]
            for x in range(1, 4):
                img[seg * x - db_cnt:seg * x + db_cnt, 0:seg] = [255, 0, 0]
        elif grid_size == [3, 3]:
            off_x = 1

        # blue play area
        img[0:seg * 3, seg * (1 - off_x):seg * (1 - off_x) + db_cnt] = [0, 255, 0]
        img[0:seg * 3, seg * (4 - off_x) - db_cnt:seg * (4 - off_x)] = [0, 255, 0]
        img[seg * 0:seg * 0 + db_cnt, seg * (1 - off_x):sy] = [0, 255, 0]
        img[seg * 3 - db_cnt:seg * 3, seg * (1 - off_x):sy] = [0, 255, 0]
        for x in range(2 - off_x, 4 - off_x):
            img[0:seg * 3, seg * x - db_cnt:seg * x + db_cnt] = [0, 255, 0]
        for x in range(1, 3):
            img[seg * x - db_cnt:seg * x + db_cnt, seg * (1 - off_x):sx] = [0, 255, 0]
