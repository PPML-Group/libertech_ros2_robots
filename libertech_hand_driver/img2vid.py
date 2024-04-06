import cv2
import os

def main():
    img_dir = f'results/hand/2024-04-05_20-55-27'
    vid_dir = img_dir
    if not os.path.exists(vid_dir):
        os.makedirs(vid_dir)
    
    out_file = os.path.join(vid_dir, f'exp.mp4')
    
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')

    # get the dimensions of the first image
    img_path = os.path.join(img_dir, os.listdir(img_dir)[0])
    img = cv2.imread(img_path)
    height, width, channels = img.shape

    # create the VideoWriter object
    out = cv2.VideoWriter(out_file, fourcc, 20, (width, height))

    # loop through the images and write them to the video
    for img_name in sorted(os.listdir(img_dir)):
        img_path = os.path.join(img_dir, img_name)
        img = cv2.imread(img_path)
        out.write(img)

    # release the VideoWriter object and close the video file
    out.release()

if __name__ == '__main__':
    main()