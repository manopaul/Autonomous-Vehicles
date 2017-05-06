# importing necessary utilities and frameworks
import os
import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import sklearn

# helper functions
def cropImage(image):
    cropped_image = image[65:140, 0:320]
    # NOTE: its img[y: y + h, x: x + w] and *not* img[x: x + w, y: y + h]
    #np.reshape(img, (height, width, nchannels)) // height =  140-65=75; width = 320-0=320; channels=3 for RGB
    resized_cropped_image = cv2.resize(cropped_image, (320,75), interpolation=cv2.INTER_AREA)
    reshaped_cropped_image = np.reshape(resized_cropped_image,(75,320,3))
    return reshaped_cropped_image

# flips the image on its horizontal axis (left/right)
def flipImage(image):
    flipped_image = np.fliplr(image)
    return flipped_image

# reverses the steering angle
def flipSteeringAngle(angle):
    flipped_steering_angle = -angle
    return flipped_steering_angle

# flips the image on its vertical axis (up/down)
def flipImageUpDown(image):
    flipped_ud_image = np.flipud(image)
    return flipped_ud_image

# converts color image to gray
def grayscale(image):
    grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return grayscale_image

# normalize the image data with Min-Max scaling to a range of [0.1, 0.9]
def normalizeGrayscaleImage(image):
    """
        """
    a = 0.1
    b = 0.9
    grayscale_min = 0
    grayscale_max = 255
    normalized_grayscale_image = a + (((image - grayscale_min)*(b - a) )/( grayscale_max - grayscale_min ))
    #print("Normalized Grayscale Image")
    #plt.imshow(normalized_grayscale_image)
    #plt.show()
    return normalized_grayscale_image

# returns the correction factor to dynamically set the steering angle
def getCorrectionFactor():
    return 0.14

# generator function to train the model using mini-batches
def generator(samples, batch_size):
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates
        sklearn.utils.shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            #local arrays to hold car images and steering angles read from the training dataset
            images = []
            angles = []
            #get steering correction factor
            angle_correction_amt = getCorrectionFactor()

            for batch_sample in batch_samples:
                #first column: image from car center camera
                center_name = 'data/IMG/'+batch_sample[0].split('/')[-1]
                center_image = cv2.imread(center_name)
                images.append(center_image)
                #augment the center image by flipping the image on its horizontal axis
                images.append(flipImage(center_image))

                #second column: image from car left camera
                left_name = 'data/IMG/'+batch_sample[1].split('/')[-1]
                left_image = cv2.imread(left_name)
                images.append(left_image)
                #augment the left image by flipping the image on its horizontal axis
                images.append(flipImage(left_image))

                #third column: image from car right camera
                right_name = 'data/IMG/'+batch_sample[2].split('/')[-1]
                right_image = cv2.imread(right_name)
                images.append(right_image)
                #augment the right image by flipping the image on its horizontal axis
                images.append(flipImage(right_image))

                #fourth column: steering angle
                steering_angle = float(batch_sample[3])

                angles.append(steering_angle)
                #augment steering angles by flipping the image on its horizontal axis
                angles.append(flipSteeringAngle(steering_angle))

                #adjust left steering angle with correction factor
                left_angle = steering_angle + angle_correction_amt
                angles.append(left_angle)
                #augment steering angles by flipping the image on its horizontal axis
                angles.append(flipSteeringAngle(left_angle))

                #adjust right steering angle with correction factor
                right_angle = steering_angle - angle_correction_amt
                angles.append(right_angle)
                #augment steering angles by flipping the image on its horizontal axis
                angles.append(flipSteeringAngle(right_angle))

            X_train = np.array(images)
            y_train = np.array(angles)

            #shuffle dataset before returning
            yield sklearn.utils.shuffle(X_train, y_train)

# read car information
def readCarData():
    print("Reading Car Data")
    with open('data/driving_log.csv') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            carData.append(row)
    print("Finished reading Car Data")
    return carData

# split the training dataset to get a test sample (0.2% of the entire training dataset)
def splitDataToTrainingAndValidationSamples(carData):
    from sklearn.model_selection import train_test_split
    #train_samples, validation_samples = train_test_split(carData, test_size=0.2)
    return train_test_split(carData, test_size=0.2)

# train the model (Keras model_version)
def trainModel(train_generator, validation_generator):
    #import Keras utilities
    from keras.models import Sequential
    from keras.layers import Flatten, Dense, Lambda, Cropping2D, Dropout, Activation
    from keras.layers.convolutional import Convolution2D
    from keras.layers.pooling import MaxPooling2D
    from keras import optimizers

    model = Sequential()
    # Preprocess incoming data, centered around zero with small standard deviation
    model.add(Lambda(lambda x: (x/127.5)-1.,input_shape=(row, col, ch), output_shape=(row, col, ch)))
    # apply cropping to reduce training time
    model.add(Cropping2D(cropping=((75,20), (1,1)),input_shape=(160, 320, 3)))
    # nvidia CNN model - 5 convolutional runs with non-linear regression
    model.add(Convolution2D(24,5,5, activation="relu", subsample=(2, 2)))
    model.add(Convolution2D(36,5,5, activation="relu", subsample=(2, 2)))
    model.add(Convolution2D(48,5,5, activation="relu", subsample=(2, 2)))
    model.add(Convolution2D(64,3,3, activation="relu", subsample=(1, 1)))
    model.add(Convolution2D(64,3,3, activation="relu", subsample=(1, 1)))
    # dropout to prevent overfitting
    model.add(Dropout(keep_prob))
    model.add(Flatten())
    model.add(Dense(100))
    model.add(Dropout(keep_prob))
    model.add(Dense(50))
    model.add(Dropout(keep_prob))
    model.add(Dense(10))
    model.add(Dropout(keep_prob))
    model.add(Dense(1))
    #leveraging the adam optimizer with a mean squared error loss
    model.compile(loss='mse', optimizer='adam')

    # saving the model
    print("Saving Model")
    model.save('model.h5')
    print("Saved Model")

    model.summary()

    #train model
    history_object = model.fit_generator(train_generator,
        samples_per_epoch=(((len(train_samples)//mini_batch_size)*mini_batch_size)*3),
        validation_data = validation_generator,
        nb_val_samples = len(validation_samples),
        nb_epoch=2, verbose=1)
    return history_object

# function to plot the model information
def plotModel(history_object):
    ### print the keys contained in the history object
    print(history_object.history.keys())

    ### plot the training and validation loss for each epoch
    plt.plot(history_object.history['loss'])
    plt.plot(history_object.history['val_loss'])
    plt.title('Model Mean Squared Error Loss')
    plt.ylabel('Mean Squared Error Loss')
    plt.xlabel('Epoch')
    plt.legend(['Training set', 'Validation set'], loc='upper right')
    plt.show()

    print("Finished Model")

# global variables
carData = []
ch, row, col = 3, 160, 320  # Trimmed image format
keep_prob = 0.9
mini_batch_size = 32

# read Car Data
carData = readCarData()
# generate training and test data set samples
train_samples, validation_samples = splitDataToTrainingAndValidationSamples(carData)

# compile and train the model using the generator function
train_generator = generator(train_samples, batch_size=mini_batch_size)
validation_generator = generator(validation_samples, batch_size=mini_batch_size)
history_object = trainModel(train_generator, validation_generator)

# visualization
plotModel(history_object)
