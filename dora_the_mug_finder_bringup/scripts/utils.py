import torch


def SaveModel(model,idx_epoch,optimizer,epoch_train_losses,epoch_test_losses,model_path,device):
    model.to('cpu')
    torch.save({
        'epoch': idx_epoch,
        'model_state_dict': model.state_dict(),
        'optimizer_state_dict': optimizer.state_dict(),
        'train_losses': epoch_train_losses,
        'test_losses': epoch_test_losses,
        }, model_path)
    model.to(device)