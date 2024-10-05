import torch 
import torch.nn as nn 
import torch.optim as optim 
from torch.utils.data import DataLoader, Dataset
from torchvision import datasets, transforms


import matplotlib.pyplot as plt

import numpy as np 
import pickle

import os 

from tqdm import tqdm

class CIFAR10(Dataset):
    #__init__ is a special method in python classes, it is called when an object is created
    def __init__(self, root, train=True, transform=None):
        #root path to the dataset, you may ignore this 
        self.root = root 
        
        #this is a boolean value to indicate whether we are loading the training or test set
        self.train = train 
        
        #this is the transformation that will be applied to the images 
        #it's none by default, you are required to pass a transform later
        self.transform = transform 

        #checking if the dataset exists, if not download it
        if not self._check_exists():
            print("Downloading cifar10 dataset")
            self.download()

        #load the data
        #We are going to load the data in memory
        #Store the images and labels in the self.data and self.targets variables
        if self.train:
            self.data, self.targets = self._load_training_data()
        else:
            self.data, self.targets = self._load_test_data()

    #this method returns the length of the dataset
    def __len__(self):
        return len(self.data)
    
    #this method returns a sample from the dataset at the given index
    #this is very important because it allows us to iterate over the dataset
    def __getitem__(self, index):
        img, target = self.data[index], int(self.targets[index])

        img = torch.from_numpy(img).float().permute(2,0,1) / 255.0

        if self.transform:
            img = self.transform(img)

        return img, target
    

    def _check_exists(self):
        return os.path.exists(os.path.join(self.root, "cifar-10-batches-py")) 

    def download(self):
        if self._check_exists():
            print("Dataset already exists !!!")
            return
        return datasets.CIFAR10(self.root, train=self.train, download=True)
    
    #this function looks complicated but it's just reading the data from the files
    def _load_batch(self, file_path):
        with open(file_path, 'rb') as f:
            data = pickle.load(f, encoding='bytes')
        return data[b'data'], data[b'labels']

    def _load_training_data(self):
        data = []
        targets = []
        for i in range(1, 6):
            file_path = os.path.join(self.root, "cifar-10-batches-py", f"data_batch_{i}")
            batch_data, batch_labels = self._load_batch(file_path)
            data.append(batch_data)
            targets.extend(batch_labels)
        
        data = np.vstack(data).reshape(-1, 3, 32, 32).transpose(0, 2, 3, 1)
        return data, np.array(targets)

    def _load_test_data(self):
        file_path = os.path.join(self.root, "cifar-10-batches-py", "test_batch")
        data, labels = self._load_batch(file_path)
        return data.reshape(-1, 3, 32, 32).transpose(0, 2, 3, 1), np.array(labels)

#We have define our datasetclass, now we are going to instantiate it
#The instantiation will download the dataset and load it in memory, it takes about 1-2 mins  
train_dataset = CIFAR10(root="data", train=True)
test_dataset = CIFAR10(root="data", train=False)

#simple demonstration __getitem__ method
img, target = train_dataset[0] #get the first image in the dataset 
plt.figure(figsize=(1,1))
plt.imshow(img.permute(1,2,0)) 
#TODO: what does permute do?, and why do we need it here? 
#answer:
# permute 方法会改变张量的维度顺序。在这里，我们需要将图像的维度从 (3, 28, 28) 转换为 (28, 28, 3)，
# 以便 matplotlib 可以正确显示图像。matplotlib 的 imshow 方法期望输入的图像维度为 (height, width, channels)。
#end of you answer 
plt.title("Original Image")
print(img.shape, "label: ",target)
#the image is a torch tensor (3, 28, 28) and the target is the label of the image


#TODO: define reasonable transformations for the images, you can use the transforms module from torchvision
transform = transforms.Compose([
    transforms.ToPILImage(),

    #TODO: Implement the transformations here 

    transforms.RandomRotation(15),
    transforms.RandomHorizontalFlip(),  # 随机水平翻转
    transforms.RandomCrop(32, padding=4),  # 随机裁剪并填充
    transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2, hue=0.1),  # 随机调整亮度、对比度、饱和度和色调
    #end of your implementation;

    transforms.ToTensor()
])

train_dataset.transform = transform

img, target = train_dataset[0] #get the first image in the dataset
plt.figure(figsize=(1,1))
plt.imshow(img.permute(1,2,0))
plt.title("Transformed Image")
print(img.shape, "label: ",target)

#apart from test set, we are going to use the training set to create a validation set
#we are going to split the training set into two parts
train_size = int(0.9 * len(train_dataset))
val_size = len(train_dataset) - train_size
train_dataset, val_dataset = torch.utils.data.random_split(train_dataset, [train_size, val_size])

#we are going to use the DataLoader class to create an iterator for our dataset
#this iterator will be used to iterate over the dataset in batches
#tentatively we are going to use a batch size of 32
#TODO: change different batch sizes and see how it affects the training process
train_loader = DataLoader(train_dataset, batch_size=128, shuffle=True, num_workers= 4, pin_memory=True)
val_loader = DataLoader(val_dataset, batch_size=128, shuffle=False,num_workers= 4, pin_memory = True)
test_loader = DataLoader(test_dataset, batch_size=128, shuffle=False,num_workers= 4, pin_memory = True)

class LinearModel(nn.Module):
    def __init__(self):
        super().__init__()
        self.name = "Linear"
        self.num_inputs = 3*32*32
        hidden_size = 512
        num_classes = 10
        super().__init__()
        self.linear = nn.Sequential(
            nn.Linear(self.num_inputs, hidden_size),    # batch_size x 784 -> batch_size x 512
            nn.ReLU(), #activation function             # batch_size x 512 -> batch_size x 512
            nn.Linear(hidden_size, num_classes)         # batch_size x 512 -> batch_size x 10
        ) #nn.Sequential is a container for other layers, it applies the layers in sequence

    #forward is the method that defines the forward pass of the network
    #not rigurously: model.forward(x) = model(x)
    def forward(self, x):
        x = x.view(-1, self.num_inputs) # flatten the image from 3x32x32 to 3072
        x = self.linear(x)
        return x
    
class CNNModel(nn.Module):
    def __init__(self):
        super().__init__()
        self.name = "CNN"
        self.conv = nn.Sequential(
            #TODO: wirte the size of the input and output of each layer e.g
            nn.Conv2d(3, 32, kernel_size=3, stride=1, padding=1), #input: 3x32x32, output: 32x32x32
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),# 输入: 32x32x32, 输出: 32x16x16
            nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1),# 输入: 32x32x32, 输出: 32x16x16
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),# 输入: 64x16x16, 输出: 64x8x8
            nn.Conv2d(64, 128, kernel_size=3, stride=1, padding=1),# 输入: 64x8x8, 输出: 128x8x8
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2)# 输入: 128x8x8, 输出: 128x4x4
        )
        self.fc = nn.Sequential(
            nn.Linear(128*4*4, 512),
            nn.ReLU(),
            nn.Linear(512, 10)
        )
    
    def forward(self, x):
        x = self.conv(x)
        x = x.view(-1, 128*4*4)
        x = self.fc(x)
        return x
    
    def getFeature(self, x):
        x = self.conv(x)
        feat = x.view(-1, 128*4*4) #this is the 128*4*4 feature 
        return feat
    
'''
class YourModel(nn.Module):
    def __init__(self):
        super().__init__() 
        self.name = "Your_model"
        #TODO: define your model here



    def forward(self, x):
        #TODO: implement the forward pass of your model


        return x
'''
#import torch.nn as nn

class YourModel(nn.Module):
    def __init__(self):
        super().__init__()
        self.name = "Your_model"
        # 定义你的模型
        self.conv = nn.Sequential(
            nn.Conv2d(3, 64, kernel_size=3, stride=1, padding=1),  # 输入: 3x32x32, 输出: 64x32x32
            nn.ReLU(),
            nn.Conv2d(64, 128, kernel_size=3, stride=1, padding=1), # 输入: 64x32x32, 输出: 128x32x32
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),  # 输入: 128x32x32, 输出: 128x16x16
            nn.Conv2d(128, 256, kernel_size=3, stride=1, padding=1), # 输入: 128x16x16, 输出: 256x16x16
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2)  # 输入: 256x16x16, 输出: 256x8x8
        )
        self.fc = nn.Sequential(
            nn.Linear(256*8*8, 1024),  # 输入: 256*8*8, 输出: 1024
            nn.ReLU(),
            nn.Linear(1024, 10)  # 输入: 1024, 输出: 10
        )

    def forward(self, x):
        # 实现你的模型的前向传播
        x = self.conv(x)
        x = x.view(-1, 256*8*8)
        x = self.fc(x)
        return x

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("using device:", device)
#instantiating the model
#TODO: change the model to the CNNModel and Your model, and evaluate the performance.
model = YourModel()
model = model.to(device)
print(f"training {model.name} model")

#defining the loss function
criterion = nn.CrossEntropyLoss()

#defining the optimizer
#TODO: try to modify the optimizer and see how it affects the training process
optimizer = optim.Adam(model.parameters(), lr=0.01)

best_accuracy = 0
#early stopping 
early_stopping = 5
early_stopping_counter = 0


#TODO: adjust the number of epochs and see how it affects the training process
epochs = 10
for epoch in range(epochs):
    #training 
    for images, labels in tqdm(train_loader):
        images = images.to(device)
        labels = labels.to(device)
        #forward pass
        outputs = model(images)
        #calculate the loss
#         print(outputs.shape)
#         print(labels.shape)
        loss = criterion(outputs, labels)
        #zero the gradients
        optimizer.zero_grad() #ensure that the gradients are zero
        #backward pass
        loss.backward()
        #optimize
        optimizer.step()
    #validation
    total = 0
    correct = 0
    for images, labels in val_loader:
        images = images.to(device)
        labels = labels.to(device)
        outputs = model(images)
        _, predicted = torch.max(outputs, 1)
        total += labels.size(0)
        correct += (predicted == labels).sum().item()
    accuracy = correct / total

    #TODO: implement early stopping
    #what is early stopping? https://en.wikipedia.org/wiki/Early_stopping
    if accuracy > best_accuracy:
        best_accuracy = accuracy
        early_stopping_counter = 0
        # 保存模型
        torch.save(model.state_dict(), f"best_model_{model.name}.pth")
        print(f"Epoch: {epoch}, Loss: {loss.item()}, Accuracy: {accuracy} ***")
    else:
        early_stopping_counter += 1
        print(f"Epoch: {epoch}, Loss: {loss.item()}, Accuracy: {accuracy}")
    
    if early_stopping_counter >= early_stopping:
        print("Early stopping triggered")
        break
    #end of early stopping

    if accuracy > best_accuracy:
        best_accuracy = accuracy
        #save the model 
        torch.save(model.state_dict(), f"best_model_{model.name}.pth")
        print(f"Epoch: {epoch}, Loss: {loss.item()}, Accuracy: {accuracy} ***")
    else: 
        print(f"Epoch: {epoch}, Loss: {loss.item()}, Accuracy: {accuracy}")

#load the best model
model.load_state_dict(torch.load(f"best_model_{model.name}.pth", weights_only=False))

#testing
total = 0
correct = 0
for images, labels in test_loader:
    images = images.to(device)
    labels = labels.to(device)
    outputs = model(images)
    _, predicted = torch.max(outputs, 1)
    total += labels.size(0)
    correct += (predicted == labels).sum().item()
accuracy = correct / total
print(f"Test Accuracy: {accuracy}")