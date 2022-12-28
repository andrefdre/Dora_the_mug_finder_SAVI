#!/usr/bin/python3

# Imports 
import argparse
import os
import glob
import random
from statistics import mean
import sys
from sklearn.model_selection import train_test_split
from torchvision import transforms
import matplotlib.pyplot as plt
from tqdm import tqdm
from colorama import Fore, Style
import torch

from dataset import Dataset
from classification_visualizer import ClassificationVisualizer
from data_visualizer import DataVisualizer
from model import Model




# Main code
def main():
    # -----------------------------------------------------------------
    # Initialization
    # -----------------------------------------------------------------
    # Argument list
    parser = argparse.ArgumentParser(description='Data Collector')
    parser.add_argument('-p', '--prev', default=True, choices=('True','False'),
                        help='Pre visualize dataset')

    arglist = [x for x in sys.argv[1:] if not x.startswith('__')]
    args = vars(parser.parse_args(args=arglist))

    # Define hyper parameters
    resume_training = False
    model_path = './model.pkl'
    device = 'cuda:0' if torch.cuda.is_available() else 'cpu' # cuda: 0 index of gpu

    model = Model() # Instantiate model

    learning_rate = 0.001
    maximum_num_epochs = 10 
    termination_loss_threshold =  0.01
    loss_function = torch.nn.CrossEntropyLoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

    dataset_path=f'{os.environ["DORA"]}/rgbd-dataset'
    image_filenames = glob.glob(dataset_path + '/*/*/*_crop.png')

    tensor_to_pill_image = transforms.Compose([
        transforms.ToPILImage()
    ])

    # Sample ony a few images for develop
    image_filenames = random.sample(image_filenames,k=700)
    train_image_filenames,test_image_filenames = train_test_split(image_filenames,test_size=0.2)

    # Creates the train dataset
    dataset_train = Dataset(train_image_filenames)
    # Creates the batch size that suits the amount of memory the graphics can handle
    loader_train = torch.utils.data.DataLoader(dataset=dataset_train,batch_size=256,shuffle=True)
    # Goes through al the images and displays them

    dataset_test = Dataset(test_image_filenames)
    loader_test = torch.utils.data.DataLoader(dataset=dataset_test, batch_size=256, shuffle=True)

    prev = args['prev']
    if prev == True:
        for image_t ,label_t in loader_train:
            num_images = image_t.shape[0]
            image_idxs=random.sample(range(0,num_images),k=25)

            fig = plt.figure()
            fig.subplots_adjust(hspace=0.5)
            for plot_idx,image_idx in enumerate(image_idxs,start=1):
                image = tensor_to_pill_image(image_t[image_idx,:,:,:]) # get images idx image_idx
                ax = fig.add_subplot(5,5,plot_idx) ## Create subplot
                ax.xaxis.set_ticklabels([])
                ax.yaxis.set_ticklabels([])
                ax.xaxis.set_ticks([])
                ax.yaxis.set_ticks([])
                label =label_t[image_idx]
                ax.set_xlabel(label)
                plt.imshow(image)

            plt.show()


    # -----------------------------------------------------------------
    # Training
    # -----------------------------------------------------------------
    # Init visualization of loss
    loss_visualizer = DataVisualizer('Loss')
    loss_visualizer.draw([0,maximum_num_epochs], [termination_loss_threshold, termination_loss_threshold], layer='threshold', marker='--', markersize=1, color=[0.5,0.5,0.5], alpha=1, label='threshold', x_label='Epochs', y_label='Loss')

    test_visualizer = ClassificationVisualizer('Test Images')

    # Resume training
    if resume_training:
        checkpoint = torch.load(model_path)
        model.load_state_dict(checkpoint['model_state_dict'])
        optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        idx_epoch = checkpoint['epoch']
        epoch_train_losses = checkpoint['train_losses']
        epoch_test_losses = checkpoint['test_losses']

        # model.train()
    else:
        idx_epoch = 0
        epoch_train_losses = []
        epoch_test_losses = []
    # -----------

    model.to(device) # move the model variable to the gpu if one exists
    while True:

        # Train batch by batch -----------------------------------------------
        train_losses = []
        for batch_idx, (image_t, label_t) in tqdm(enumerate(loader_train), total=len(loader_train), desc=Fore.GREEN + 'Training batches for Epoch ' + str(idx_epoch) +  Style.RESET_ALL):

            image_t = image_t.to(device)
            label_t = label_t.to(device)

            # Apply the network to get the predicted ys
            label_t_predicted = model.forward(image_t)
 
            # Compute the error based on the predictions
            loss = loss_function(label_t_predicted, label_t)

            # Update the model, i.e. the neural network's weights 
            optimizer.zero_grad() # resets the weights to make sure we are not accumulating
            loss.backward() # propagates the loss error into each neuron
            optimizer.step() # update the weights


            train_losses.append(loss.data.item())

        # Compute the loss for the epoch
        epoch_train_loss = mean(train_losses)
        epoch_train_losses.append(epoch_train_loss)

        # Run test in batches ---------------------------------------
        # TODO dropout
        test_losses = []
        for batch_idx, (image_t, label_t) in tqdm(enumerate(loader_test), total=len(loader_test), desc=Fore.GREEN + 'Testing batches for Epoch ' + str(idx_epoch) +  Style.RESET_ALL):

            image_t = image_t.to(device)
            label_t = label_t.to(device)

            # Apply the network to get the predicted ys
            label_t_predicted = model.forward(image_t)

            # Compute the error based on the predictions
            loss = loss_function(label_t_predicted, label_t)

            test_losses.append(loss.data.item())

            # test_visualizer.draw(image_t, label_t, label_t_predicted)

        # Compute the loss for the epoch
        epoch_test_loss = mean(test_losses)
        epoch_test_losses.append(epoch_test_loss)

        # Visualization
        loss_visualizer.draw(list(range(0, len(epoch_train_losses))), epoch_train_losses, layer='train loss', marker='-', markersize=1, color=[0,0,0.7], alpha=1, label='Train Loss', x_label='Epochs', y_label='Loss')

        loss_visualizer.draw(list(range(0, len(epoch_test_losses))), epoch_test_losses, layer='test loss', marker='-', markersize=1, color=[1,0,0.7], alpha=1, label='Test Loss', x_label='Epochs', y_label='Loss')

        loss_visualizer.recomputeAxesRanges()

        print(Fore.BLUE + 'Epoch ' + str(idx_epoch) + ' Loss ' + str(epoch_train_loss) + Style.RESET_ALL)

        # Save checkpoint
        model.to('cpu')
        torch.save({
            'epoch': idx_epoch,
            'model_state_dict': model.state_dict(),
            'optimizer_state_dict': optimizer.state_dict(),
            'train_losses': epoch_train_losses,
            'test_losses': epoch_test_losses,
            }, model_path)
        model.to(device)

        idx_epoch += 1 # go to next epoch
        # Termination criteria
        if idx_epoch > maximum_num_epochs:
            print('Finished training. Reached maximum number of epochs.')
            break
        elif epoch_train_loss < termination_loss_threshold:
            print('Finished training. Reached target loss.')
            break

if __name__ == '__main__':
    main()
