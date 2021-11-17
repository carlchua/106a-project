#!/usr/bin/env python
"""Segmentation skeleton code for Lab 6
Course: EECS C106A, Fall 2019
Author: Grant Wang

This Python file is the skeleton code for Lab 3. You are expected to fill in
the body of the incomplete functions below to complete the lab. The 'test_..'
functions are already defined for you for allowing you to check your
implementations.

When you believe you have completed implementations of all the incompeleted
functions, you can test your code by running python segmentation.py at the
command line and step through test images
"""

import os
import numpy as np
import cv2
import matplotlib.pyplot as plt
# import matplotlib
# matplotlib.use('Agg')

this_file = os.path.dirname(os.path.abspath(__file__))
IMG_DIR = '/'.join(this_file.split('/')[:-2]) + '/img'

def read_image(img_name, grayscale=False):
    """ reads an image

    Parameters
    ----------
    img_name : str
        name of image
    grayscale : boolean
        true if image is in grayscale, false o/w

    Returns
    -------
    ndarray
        an array representing the image read (w/ extension)
    """

    if not grayscale:
        img = cv2.imread(img_name)
    else:
        img = cv2.imread(img_name, 0)

    return img

def write_image(img, img_name):
    """writes the image as a file

    Parameters
    ----------
    img : ndarray
        an array representing an image
    img_name : str
        name of file to write as (make sure to put extension)
    """

    cv2.imwrite(img_name, img)

def show_image(img_name, title='Fig', grayscale=False):
    """show the  as a matplotlib figure

    Parameters
    ----------
    img_name : str
        name of image
    tile : str
        title to give the figure shown
    grayscale : boolean
        true if image is in grayscale, false o/w
    """

    if not grayscale:
        plt.imshow(img_name)
        plt.title(title)
        plt.show()
    else:
        plt.imshow(img_name, cmap='gray')
        plt.title(title)
        plt.show()


def threshold_segment_naive(gray_img, lower_thresh, upper_thresh):
    """perform grayscale thresholding using a lower and upper threshold by
    blacking the background and whitening lying between the threholds (the
    foreground)

    Parameter
    ---------
    gray_img : ndarray
        grayscale image array
    lower_thresh : float or int
        lowerbound to threshold (an intensity value between 0-255)
    upper_thresh : float or int
        upperbound to threshold (an intensity value between 0-255)

    Returns
    -------
    ndarray
        thresholded version of gray_img
    """
    # TODO: Implement threshold segmentation by setting pixels of gray_img inside the
    # lower_thresh and upper_thresh parameters to 1
    # Then set any value that is outside the range to be 0
    # Hints: make a copy of gray_img so that we don't alter the original image
    # Boolean array indexing, or masking will come in handy.
    # See https://docs.scipy.org/doc/numpy-1.13.0/user/basics.indexing.html

    new_img = gray_img.copy()
    for i in range(new_img.shape[0]):
        for j in range(new_img.shape[1]):
            pixel = new_img[i][j]
            if pixel >= lower_thresh and pixel <= upper_thresh:
                new_img[i][j] = 1
            else:
                new_img[i][j] = 0
    return new_img

def threshold_segment_naive2(color_img, lt0, ut0, lt1, ut1, lt2, ut2):
    """perform grayscale thresholding using a lower and upper threshold by
    blacking the background and whitening lying between the threholds (the
    foreground)

    Parameter
    ---------
    gray_img : ndarray
        grayscale image array
    lower_thresh : float or int
        lowerbound to threshold (an intensity value between 0-255)
    upper_thresh : float or int
        upperbound to threshold (an intensity value between 0-255)

    Returns
    -------
    ndarray
        thresholded version of gray_img
    """
    # TODO: Implement threshold segmentation by setting pixels of gray_img inside the
    # lower_thresh and upper_thresh parameters to 1
    # Then set any value that is outside the range to be 0
    # Hints: make a copy of gray_img so that we don't alter the original image
    # Boolean array indexing, or masking will come in handy.
    # See https://docs.scipy.org/doc/numpy-1.13.0/user/basics.indexing.html

    new_img = color_img.copy()
    new_img = cv2.GaussianBlur(new_img, (19, 19), 0)
    show_image(new_img, title='img_blurred')
    img_masked = np.zeros((new_img.shape[0], new_img.shape[1]))
    for i in range(new_img.shape[0]):
        for j in range(new_img.shape[1]):
            pixel = new_img[i][j]
            r,g,b = int(pixel[0]), int(pixel[1]), int(pixel[2])
            if r>=lt0 and r<=ut0 and g>=lt1 and g<=ut1 and b>=lt2 and b<=ut2:
                img_masked[i][j] = 1
                sim = 50
                if np.absolute(r-g)<=sim and np.absolute(g-b)<=sim and np.absolute(r-b)<=sim:
                        img_masked[i][j] = 0
            # else:
            #     img_masked[i][j] = 0
    return img_masked

def edge_detect_naive(gray_img):
    """perform edge detection using first two steps of Canny (Gaussian blurring and Sobel
    filtering)

    Parameter
    ---------
    gray_img : ndarray
        grayscale image array

    Returns
    -------
    ndarray
        gray_img with edges outlined
    """

    gray_s = gray_img.astype('int16') # convert to int16 for better img quality
    # TODO: Blur gray_s using Gaussian blurring, convole the blurred image with
    # Sobel filters, and combine to compute the intensity gradient image (image with edges highlighted)
    # Hints: open-cv GaussianBlur will be helpful https://medium.com/analytics-vidhya/gaussian-blurring-with-python-and-opencv-ba8429eb879b
    # Use opencv's filter2D to perform the convolution.

    # Steps
    # 1. apply a gaussian blur with a 5x5 kernel.
    # 2. define the convolution kernel Kx and Ky as defined in the doc.
    # 3. compute Gx and Gy by convolving Kx and Ky respectively with the blurred image.
    # 4. compute G = sqrt(Gx ** 2 + Gy ** 2)
    # 5. Return G

    # Comment out this line after you implement the function.
    #raise NotImplementedError()

    # First, apply a gaussian blur with a 5x5 kernel.
    blur = cv2.GaussianBlur(gray_s, (5, 5), 0)
    # Next, define the convolution kernel Kx and Ky as defined in the doc.
    Kx = np.array([[-1, -2, -1],
                     [0, 0, 0],
                     [1, 2, 1]])
    Ky = Kx.T

    # Now, compute Gx and Gy by convolving Kx and Ky respectively with the blurred image.
    G_x = np.array(cv2.filter2D(blur, -1, Kx)).astype(np.int64)
    G_y = np.array(cv2.filter2D(blur, -1, Ky)).astype(np.int64)

    # Finally, compute G = sqrt(Gx ** 2 + Gy ** 2)
    # If you have errors computing the square root, you may be encountering overflow - change datatype to int64
    G = np.sqrt(G_x ** 2 + G_y ** 2)

    # Return G
    return G


def edge_detect_canny(gray_img):
    """perform Canny edge detection

    Parameter
    ---------
    gray_img : ndarray
        grayscale image array

    Returns
    -------
    ndarray
        gray_img with edges outlined
    """

    edges = cv2.Canny(gray_img, 100, 200)

    return edges

def do_kmeans(data, n_clusters):
    """Uses opencv to perform k-means clustering on the data given. Clusters it into
       n_clusters clusters.

       Args:
         data: ndarray of shape (n_datapoints, dim)
         n_clusters: int, number of clusters to divide into.

       Returns:
         clusters: integer array of length n_datapoints. clusters[i] is
         a number in range(n_clusters) specifying which cluster data[i]
         was assigned to.
    """
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.2)
    _, clusters, centers = kmeans = cv2.kmeans(data.astype(np.float32), n_clusters, bestLabels=None, criteria=criteria, attempts=1, flags=cv2.KMEANS_RANDOM_CENTERS)

    return clusters

def cluster_segment(img, n_clusters, random_state=0):
    """segment image using k_means clustering

    Parameter
    ---------
    img : ndarray
        rgb image array
    n_clusters : int
        the number of clusters to form as well as the number of centroids to generate
    random_state : int
        determines random number generation for centroid initialization

    Returns
    -------
    ndarray
        clusters of gray_img represented with similar pixel values
    """
    # Remove this line when you implement this function.
    #raise NotImplementedError()

    # Downsample img by a factor of 2 first using the mean to speed up K-means
    img_d = cv2.resize(img, dsize=(img.shape[1]/2, img.shape[0]/2), interpolation=cv2.INTER_NEAREST)
    a = img_d.shape
    # TODO: Generate a clustered image using K-means

    # first convert our 3-dimensional img_d array to a 2-dimensional array
    # whose shape will be (height * width, number of channels) hint: use img_d.shape
    img_r = np.reshape(img_d, (img_d.shape[0]*img_d.shape[1], img_d.shape[2])) #ODO

    # fit the k-means algorithm on this reshaped array img_r using the
    # the do_kmeans function defined above.
    clusters = do_kmeans(img_r, n_clusters)

    # reshape this clustered image to the original downsampled image (img_d) width and height
    cluster_img = np.reshape(clusters, (a[0], a[1]))

    # Upsample the image back to the original image (img) using nearest interpolation
    img_u = cv2.resize(src=cluster_img, dsize=(img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)

    return img_u.astype(np.uint8)


def to_grayscale(rgb_img):
    return np.dot(rgb_img[... , :3] , [0.299 , 0.587, 0.114])

def segment_image(img):
    # ONLY USE ONE SEGMENTATION METHOD

    # perform thresholding segmentation
    #binary = threshold_segment_naive(to_grayscale(img), 50, 65).astype(np.uint8)

    ### A1 SOCCER ###
    binary = threshold_segment_naive2(to_grayscale(img), 0,130, 70,220, 100,220).astype(np.uint8)

    # perform clustering segmentation.
    #binary = cluster_segment(img, 4).astype(np.uint8)

    if np.mean(binary) > 0.5:
        binary = 1 - binary #invert the pixels if K-Means assigned 1's to background, and 0's to foreground

    return binary

"""
below are tests used for sanity checking you image, edit as you see appropriate

"""

def test_thresh_naive(img, lower_thresh, upper_thresh):
    thresh = threshold_segment_naive(img, lower_thresh, upper_thresh)
    show_image(thresh, title='thresh naive', grayscale=True)
    cv2.imwrite(IMG_DIR + "/thresh.jpg", thresh.astype('uint8') * 255)

def test_thresh_naive2(img, lt0,ut0,lt1,ut1,lt2,ut2):
    show_image(img, title='img')
    thresh2 = threshold_segment_naive2(img, lt0,ut0,lt1,ut1,lt2,ut2)
    show_image(thresh2, title='thresh naive2')
    cv2.imwrite(IMG_DIR + "/thresh2.jpg", thresh2.astype('uint8') * 255)

def test_edge_naive(img):
    edges = edge_detect_naive(img)
    show_image(edges, title='edge naive', grayscale=True)
    cv2.imwrite(IMG_DIR + "/edges.jpg", edges)

def test_edge_canny(img):
    edges = edge_detect_canny(img)
    show_image(edges, title='edge canny', grayscale=True)
    cv2.imwrite(IMG_DIR + "/edges_canny.jpg", edges)

def test_cluster(img, n_clusters):
    # For visualization, we need to scale up the image so it
    # is in range(256) instead of range(n_clusters).
    clusters = (cluster_segment(img, n_clusters) * (255 / (n_clusters-1))).astype(np.uint8)

    cv2.imwrite(IMG_DIR + "/cluster.jpg", clusters)
    clusters = cv2.imread(IMG_DIR + '/cluster.jpg')
    show_image(clusters, title='cluster')

if __name__ == '__main__':
    # adjust the file names here
    test_img = read_image(IMG_DIR + '/staples.jpg', grayscale=True)
    test_img_color = read_image(IMG_DIR + '/staples.jpg')

    soccer_test_img_color0 = read_image(IMG_DIR + '/soccerball_screenshot0.jpg')
    soccer_test_img_color1 = read_image(IMG_DIR + '/soccerball_screenshot1.jpg')
    soccer_test_img_color2 = read_image(IMG_DIR + '/soccerball_screenshot2.jpg')
    soccer_test_img_color3 = read_image(IMG_DIR + '/soccerball_screenshot3.jpg')
    soccer_test_img_color4 = read_image(IMG_DIR + '/soccerball_screenshot4.jpg')

    # uncomment the test you want to run
    # it will plot the image and also save it

    #test_thresh_naive(test_img, 100, 250)
    test_thresh_naive2(soccer_test_img_color0, 0,130, 70,220, 100,220)
    test_thresh_naive2(soccer_test_img_color1, 0,130, 70,220, 100,220)
    test_thresh_naive2(soccer_test_img_color2, 0,130, 70,220, 100,220)
    test_thresh_naive2(soccer_test_img_color3, 0,130, 70,220, 100,220)
    test_thresh_naive2(soccer_test_img_color4, 0,130, 70,220, 100,220)

    # test_edge_naive(test_img)
    # test_edge_canny(test_img)
    #test_cluster(test_img_color, 6)
