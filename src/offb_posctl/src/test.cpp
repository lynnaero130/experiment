#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <Eigen/Dense>
using namespace std;

int ReadRowNumber(string fileway);
void Readdata(string fileway,vector<vector<double>> &ans);

int main(int argc, char **argv)//argc  argument count 传参个数，argument value
{
    ifstream fin_u1;// the instance can be used to read the file
    string fileway = "/home/uav/lzy_ws/src/offb_posctl/data/line.csv";
    int r = ReadRowNumber(fileway);
    vector<vector<double>> trajectory(r, vector<double>(3, 0));
    Readdata(fileway,trajectory);

    for(int i=0;i<r;++i){
        for(int j=0;j<3;++j){
            cout<<trajectory[i][j]<<"   ";
        }
        cout<<endl;
    }
}

int ReadRowNumber(string fileway)
{
    ifstream fin_u1;// the instance can be used to read the file
    string line; //used for store the one line data from csv temporarily
    int r = 0;
    fin_u1.open(fileway);// the directory of th data.csv
    while (!fin_u1.eof()) {// juddge whether the fin reach the end line of the csv file
        fin_u1 >> line;// read one line data to "line"
        fin_u1 >> line;// read one line data to "line"
        fin_u1 >> line;// read one line data to "line"
        r++;
    }
    fin_u1.close();
    return r;
}
void Readdata(string fileway,vector<vector<double>> &ans)
{
    ifstream fin_u1;// the instance can be used to read the file
    string line; //used for store the one line data from csv temporarily
    int index = 0;
    fin_u1.open(fileway);// the directory of th data.csv
    while (!fin_u1.eof())
    {// juddge whether the fin reach the end line of the csv file
        fin_u1 >> line;// read one line data to "line"
        ans[index][0]= atof(line.c_str());
        fin_u1 >> line;
        ans[index][1]=atof(line.c_str());
        fin_u1 >> line;
        ans[index][2]=atof(line.c_str());
        cout<<ans[index][2]<<endl;
        index++;
    }
}