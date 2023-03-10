import torch
import matplotlib.pyplot as plt
import os
import glob


def SaveModel(model,idx_epoch,optimizer,training_loader,testing_loader,epoch_train_losses,epoch_test_losses,model_path,device):
    model.to('cpu')
    torch.save({
        'epoch': idx_epoch,
        'model_state_dict': model.state_dict(),
        'optimizer_state_dict': optimizer.state_dict(),
        'loader_train': training_loader,
        'loader_test': testing_loader,
        'train_losses': epoch_train_losses,
        'test_losses': epoch_test_losses,
        }, model_path)
    model.to(device)

def LoadModel(model_path,model,device):
    checkpoint = torch.load(model_path)
    model.load_state_dict(checkpoint['model_state_dict'])
    model.to(device) # move the model variable to the gpu if one exists
    return model

def SaveGraph(train_losses,test_losses,folder_name):
    plt.figure()
    plt.plot(train_losses, label='train loss')
    plt.plot(test_losses, label='test loss')
    plt.xlabel("Epoch")    
    plt.ylabel("Loss")   
    plt.legend()
    plt.savefig(f'{folder_name}/losses.png')

def GetClassListFromFolder():
    dataset_path=f'{os.environ["DORA"]}/rgbd-dataset'
    folder_names = glob.glob(dataset_path + '/*')
    classList=[]
    for folder_name in folder_names:
        parts = folder_name.split('/')
        part = parts[-1]
        classList.append(part)
    return classList
 