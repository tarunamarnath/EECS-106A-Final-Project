import cv2
import matplotlib.pyplot as plt
import numpy as np
import skimage.transform as sktr
import torch
from skimage import exposure
import skimage.io as skio
from torch import nn
from torch.utils.data import Dataset


class Net(nn.Module):
    def __init__(self):
        super().__init__()

        self.encoder = nn.Sequential(
            nn.Conv2d(1, 24, 3, 1, padding=2),
            nn.ReLU(),
            nn.AvgPool2d(kernel_size=2, stride=2, padding=0),
            nn.Conv2d(24, 24, 3, 1, padding=2),
            nn.ReLU(),
            nn.AvgPool2d(kernel_size=2, stride=2, padding=0),
            nn.Conv2d(24, 32, 3, 1, padding=2),
            nn.ReLU(),
            nn.AvgPool2d(kernel_size=2, stride=2, padding=0),
            nn.Conv2d(32, 32, 3, 1),
            nn.ReLU(),
            nn.AvgPool2d(kernel_size=2, stride=2, padding=0),
            nn.Conv2d(32, 32, 2, 1, padding=2),
            nn.ReLU(),
            nn.AvgPool2d(kernel_size=2, stride=2, padding=0),
            nn.Conv2d(32, 64, 2, 1),
            nn.ReLU(),
        )

        self.decoder = nn.Sequential(
            nn.Linear(9216, 5120),
            nn.ReLU(),
            nn.Linear(5120, 16),
            nn.Sigmoid()
        )

    def forward(self, x):
        batch_size = x.shape[0]
        features = self.encoder(x).squeeze()

        product = np.prod(np.array(features.shape))
        features = features.reshape(batch_size, product // batch_size)

        return self.decoder(features).reshape(batch_size, 8, 2)


model = torch.load("./src/model.pth", map_location=torch.device('cpu'))
model.eval()


class Table(Dataset):
    """Table Landmarks dataset."""

    def __init__(self, image):
        """
        Args:
            root_dir (string): Directory with all the data.
            transform (callable, optional): Optional transform to be applied
                on a sample.
        """
        self.image = image
        self.len = 1

    def __len__(self):
        return self.len

    def transform(self, sample):
        sample['image'] = sktr.resize(sample['image'], (300, 500))
        sample['image'] = exposure.equalize_adapthist(sample['image'], clip_limit=0.005)

        return sample

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()

        sample = {'image': self.image[..., 1], 'landmarks': np.array([])}

        sample = self.transform(sample)

        return sample


def get_points(image):
    '''Predicts 8 points corresponding to AR Tag and Towel given an image'''
    dataset = Table(image)
    dataloader = torch.utils.data.DataLoader(dataset)

    with torch.no_grad():
        for datapoint in dataloader:
            x, y = datapoint['image'], datapoint['landmarks']
            x, y = x.float(), y.float()
            x = torch.reshape(x, (x.shape[0], 1, x.shape[1], x.shape[2]))
            pred = model(x)
            img = datapoint['image']
            a, b = pred.T
            a = a * img.shape[2]
            b = b * img.shape[1]

            return a, b, pred.squeeze().numpy()


def pad(arr):
    '''Pads an array along the Z-axis'''
    ones = np.ones((arr.shape[0], 1)).astype(int)
    return np.hstack((arr, ones))


def transform(H, im1):
    '''Applies a Homography matrix to the points'''
    if im1.shape[1] < 3:
        im1 = pad(im1)
    transformation = H @ im1.T
    return (transformation[:2] / transformation[2]).T


def real_world_points(image):
    '''Retrieves Translation from center of AR Tag to coordinates'''
    a, b, points = get_points(image)
    points = points * (image.shape[1], image.shape[0])

    bot_x, bot_y = 350, 150
    dst_pts = np.array([
        [bot_x, bot_y],
        [bot_x, bot_y - 55],
        [bot_x + 55, bot_y - 52.5],
        [bot_x + 55, bot_y - 50 + 50]
    ])

    H, _ = cv2.findHomography(pad(points[4:]), pad(dst_pts), 0)

    h, w = 300, 500
    indy, indx = np.indices((300, 500), dtype=np.float32)
    lin_homg_ind = np.array([indx.ravel(), indy.ravel(), np.ones_like(indx).ravel()])

    map_x, map_y = transform(np.linalg.inv(H), lin_homg_ind.T).T  # ensure homogeneity
    map_x = map_x.reshape(h, w).astype(np.float32)
    map_y = map_y.reshape(h, w).astype(np.float32)
    dst1 = cv2.remap(image, map_x, map_y, cv2.INTER_LINEAR)

    tag, towel = transform(H, pad(points[4:])), transform(H, pad(points[:4]))
    x, y = towel.T
    m, n = tag.T

    tag_center = np.mean(tag, axis=0)

    tag, towel = tag - tag_center, towel - tag_center

    plt.scatter(x, y, marker='o', c="b")
    plt.scatter(m, n, marker='o', c="g")

    plt.imshow(dst1)
    plt.show()

<<<<<<< HEAD
    return tag, towel


def get_translation_vectors(image_path):
    image = skio.imread(image_path)
    return real_world_points(image)
=======
    return towel / 10, tag / 10


def get_translation_vectors(image):
    return real_world_points(image)

# if __name__ == "__main__":
#     im = skio.imread("./src/img/img27.png")
#     x, y = real_world_points(im)
#
#     plt.show()
>>>>>>> 5f801bb258f2795d58098dae5e519f0119735fa3
