import numpy as np
from autolab_core import RigidTransform, YamlConfig

class Circle:
    def __init__(self):
        pass
    @staticmethod
    def grasp_symmetries():
        return np.array([-np.pi, -np.pi/2, 0, np.pi/2, np.pi])
    def placement_symmetries():
        return np.linspace(-np.pi,np.pi,5)
    @staticmethod
    def tforms_to_pose(ids, tforms,goal=False):
        if goal:
            shape_specific_ids = [12,13]
        else:
            shape_specific_ids = [10,11]
        relevant_ids = [ar_id for ar_id in ids if ar_id in shape_specific_ids]
        relevant_tforms = []
        for id_num in relevant_ids:
            tform_idx = ids.index(id_num)
            relevant_tforms.append(tforms[tform_idx])

        #avg_rotation = relevant_tforms[0]
        #for i in range(1, len(relevant_tforms)):
        #    avg_rotation = avg_rotation.interpolate_with(relevant_tforms[i], 0.5)

        if len(relevant_ids) == 2:
            translation = np.mean(np.vstack([T.translation for T in relevant_tforms]), axis=0)
        else:
            print(relevant_ids)
            print("Not enough detections to make accurate pose estimate for circle ")
            return None
        return RigidTransform(rotation = np.eye(3), translation = translation, to_frame = tforms[0].to_frame, from_frame = "peg_center")
class Obstacle:
    def __init__(self):
        pass
    @staticmethod
    def symmetries():
        return [-np.pi,-np.pi/2, 0, np.pi/2, np.pi]
    @staticmethod
    def tforms_to_pose(ids, tforms,goal=False):
        shape_specific_ids = [8,9]
        relevant_ids = [ar_id for ar_id in ids if ar_id in shape_specific_ids]
        relevant_tforms = []
        for id_num in relevant_ids:
            tform_idx = ids.index(id_num)
            relevant_tforms.append(tforms[tform_idx])
        
        avg_rotation = relevant_tforms[0]
        for i in range(1, len(relevant_tforms)):
            avg_rotation = avg_rotation.interpolate_with(relevant_tforms[i], 0.5)

        if len(relevant_ids) == 2:
            translation = np.mean(np.vstack([T.translation for T in relevant_tforms]), axis=0)
        else:
            print(relevant_ids)
            print("Not enough detections to make accurate pose estimate")
            return None
        return RigidTransform(rotation = avg_rotation.rotation, translation = translation, to_frame = tforms[0].to_frame, from_frame = "peg_center")

class Rectangle:
    def __init__(self):
       """
        Assumes d distance from the center of the tag to the sticker. Then this 
            w
        ----------
       |0       1 |
       |          |
       |          |  l
       |3       2 |
        ----------
       """
        self.d = 0.005 #distance between center of tag and the edge
        self.w = 0.1
        self.l = 0.2

    @staticmethod
    def symmetries():
        return np.array([-np.pi, 0, np.pi])
    def grasp_symmetries():
        return np.array([-np.pi, -np.pi/2, 0, np.pi/2, np.pi])
    def placement_symmetries():
        return np.array([0, -np.pi, np.pi, 2*np.pi])

    @staticmethod
    def tforms_to_pose(ids, tforms,goal=False):
        """
        returns the intervals of rotation that are identical, at least enough to be useful. 
        The triangle can't be rotated but the rectangle can be inserted at any interval of 3.14


        """
        shape_specific_ids = [0,1,2,3]
        id_to_transform = {}
        for id_num in ids:
            if id_num in shape_specific_ids:
                id_to_transform[id_num] = tforms[id_num]

        if len(id_to_transform.keys() == 0:
            print(ids)
            print("Not enough detections to make accurate pose estimate")
            return None

        tfs_detected = id_to_transform.values()
        avg_rotation = tfs_detected[0]
        for i in range(1, len(tfs_detected)):
            avg_rotation = avg_rotation.interpolate_with(tfs_detected[i], 0.5)

        pose_estimates = []
        for id_num in id_to_transform.keys():
            T = id_to_transform[id_num].translation
            if id_num == 0:
                pose_estimates.append([T[0]+(0.5*self.l-d), T[1]+(0.5*self.w-d), T[2]])
            elif id_num == 1:
                pose_estimates.append([T[0]+(0.5*self.l-d), T[1]-(0.5*self.w-d), T[2]])
            elif id_num == 2:
                pose_estimates.append([T[0]-(0.5*self.l-d), T[1]-(0.5*self.w-d), T[2]])
            elif id_num == 3:
                pose_estimates.append([T[0]-(0.5*self.l-d), T[1]+(0.5*self.w-d), T[2]])
            else:
                raise ValueError("Invalid id num. Should not come here")
        translation = np.mean(np.vstack(pose_estimates), axis=0)
        return RigidTransform(rotation = relevant_tforms[0].rotation, translation = translation, to_frame = tforms[0].to_frame, from_frame = "peg_center")

class Square:
    def __init__(self):
        pass
    def grasp_symmetries():
        return np.array([-np.pi, -np.pi/2, 0, np.pi/2, np.pi])
    def placement_symmetries():
        return np.array([0, -np.pi, np.pi, 2*np.pi, np.pi/2, -np.pi/2, 1.5*np.pi, -1.5*np.pi])

    """
    returns the intervals of rotation that are identical, at least enough to be useful. 
    The triangle can't be rotated but the rectangle can be inserted at any interval of 3.14
    """
    @staticmethod
    def tforms_to_pose(ids, tforms,goal=False):
        if goal:
            shape_specific_ids = [18,19, 20, 21]
        else:
            shape_specific_ids = [14,15,16,17]

        relevant_ids = [ar_id for ar_id in ids if ar_id in shape_specific_ids]
        relevant_tforms = []
        for id_num in relevant_ids:
            tform_idx = ids.index(id_num)
            relevant_tforms.append(tforms[tform_idx])

        avg_rotation = relevant_tforms[0]
        for i in range(1, len(relevant_tforms)):
            avg_rotation = avg_rotation.interpolate_with(relevant_tforms[i], 0.5)

        if len(relevant_ids) == 4:
            translation = np.mean(np.vstack([T.translation for T in relevant_tforms]), axis=0)
        corner_pairs = [(14,16), (15,17), (18,20),(19,21)]
        for pair in corner_pairs:
            if pair[0] in relevant_ids and pair[1] in relevant_ids:
                first_idx = ids.index(pair[0])
                second_idx = ids.index(pair[1])
                relevant_tforms = [tforms[first_idx], tforms[second_idx]]
                translation = np.mean(np.vstack([T.translation for T in relevant_tforms]), axis=0)

        if translation is None:
            print(relevant_ids)
            print("Not enough detections to make accurate pose estimate")
            return None
        return RigidTransform(rotation = relevant_tforms[0].rotation, translation = translation, to_frame = tforms[0].to_frame, from_frame = "peg_center")
