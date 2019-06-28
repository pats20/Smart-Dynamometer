# Smart-Dynamometer
Modeling and Characterization of RC-scale Smart Dynamometer for Electric Vehicle Testing


## Goal
Goal is to predict if a passenger survived the sinking of the Titanic or not. For each in the test set, you must predict a 0 or 1 value for the variable.

## Metric
Find the accuracy scores of the models used.

## Submission File Format
A CSV file with exactly 418 entries plus a header row. 

### Format of CSV file
The file should have exactly 2 columns:

PassengerId (sorted in any order)
Survived (contains your binary predictions: 1 for survived, 0 for deceased)
PassengerId,Survived
 892,0
 893,1
 894,0
 Etc.

## Project Phase and Planning

![alt text](https://github.com/vinsingh29/Titanic-Kaggle-Competition/blob/master/docs/Project%20Planning.png)

## Alogorithms Used 


```
1.Random Forests Model
2.Support Vector Machines
3.Gradient Boosting Classifier
4.K-nearest neighbors
5.Gaussian Naive Bayes
6.Logistic Regression
7.Decision Tree

```

### Results and Conclusion

```

Test       logistic	      rf	      svc	     knn
test1	0.797980	0.809203	0.745230	0.747475
test2	0.791246	0.818182	0.764310	0.757576
test3	0.796857	0.812570	0.763187	0.757576
test4	0.796857	0.810325	0.832772	0.778900
test5	0.794613	0.812570	0.760943	0.741863
test6	0.790123	0.823793	0.831650	0.775533

```
![alt text](https://github.com/vinsingh29/Titanic-Kaggle-Competition/blob/master/docs/Result%20of%20Approach1.png)


## Authors

* **Team CMPE 255** 


## Acknowledgments

* Thanks Prof. Sithu Aung for all your guidance and help.


