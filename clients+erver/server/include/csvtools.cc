/* -------------------------------------------------------------------------- *
 *                           csvtools.cpp                                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Matthew Millard                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
#include "csvtools.h"

int readMatrixFromFile(const std::string& filename,
                         std::vector<std::vector<mjtNum > > &dataMatrix,
                         bool withheader,
                         std::vector<std::string>* header)
{
    std::ifstream datafile;
	datafile.open(filename.c_str());       
    int row = 0;
    
    //SimTK::Matrix data;
    //std::vector<std::vector<double > > dataMatrix;
    std::vector<mjtNum > rowVector;

    if(datafile.is_open())
    {
        std::string line;
        std::string entry;
        std::string delimiter = ",";
        //int matrixRowNum = 0;
        int matrixColNum = 1;
        int startingRow = (withheader)?1:0;

        //1. Size the matrix
        getline(datafile,line); //get a line of text
        if(withheader)
        {
            if(header)
            {
                size_t pos = 0;
                std::string token;
                while ((pos = line.find(delimiter)) != std::string::npos) {
                    token = line.substr(0, pos);
                    //std::cout << token << std::endl;
                    line.erase(0, pos + delimiter.length());
                    if(header)(*header).push_back(token);
            }
            }
            getline(datafile,line);
            row++;
        }

        //parse it for all of the comma spots
        size_t pos1 = 0;
        size_t pos2 = 0;
        do{                    
            pos2 = line.find(delimiter,pos1);
            //if this is the first time running this loop, count
            //the number of columns
            if(pos2 != std::string::npos)
                matrixColNum++;

            pos1 = pos2+1;
            //printf("pos2=%d\n",pos2);
        }while(pos2 != std::string::npos);

        //Initial matrix sizing
        rowVector.resize(matrixColNum);

        //printf("matrixColNum=%d\n",matrixColNum);

        while(datafile.good())
        {
            pos1 = 0;
            pos2 = 0;          
            for(int i=0; i < matrixColNum; i++){
                pos2 = line.find(delimiter,pos1);
                if(pos2 == std::string::npos)
                    pos2 = line.length();
                entry = line.substr(pos1,pos2-pos1);
                pos1 = pos2+1;
                //data(row,i) = atof(entry.c_str());
                rowVector[i] = atof(entry.c_str());
            }

            //Resize the matrix if its too small for the next line
            //if(row == matrixRowNum-1){
            //    matrixRowNum = matrixRowNum*2;
            //    data.resizeKeep(matrixRowNum,matrixColNum);
            //}
            dataMatrix.push_back(rowVector);

            row++;
            getline(datafile,line);
        }
            //data.resizeKeep(row,matrixColNum);
        
    }
    else
    {
        std::cout << "csvtools error: open file failed! " << std::endl;
    }
	datafile.close();
    //return dataMatrix;
    return (withheader)?row-1:row;
}

void printMatrixToFile( 
    const std::vector<std::vector<mjtNum > >& dataMatrix, 
    const std::string& header, 
    const std::string& filename)
{
	std::cout << "write " << filename << std::endl;
    std::ofstream datafile;
	datafile.open(filename.c_str());
   // datafile << std::scientific;
   // datafile.precision(16);
    if(header.length() > 1)
        datafile << header << "\n";

  for(unsigned int i = 0; i < dataMatrix.size(); i++){
    for(unsigned int j = 0; j < dataMatrix[0].size(); j++){
			if(j<dataMatrix[0].size()-1){
				datafile << dataMatrix[i][j] << ", ";
            }
			else{
                datafile << dataMatrix[i][j] << "\n";
            }            
		}	
	}
	datafile.close();
}

void printMatrixToFile(
    const std::vector<std::vector< int > >& dataMatrix,
    const std::string& header,
    const std::string& filename)
{

    std::ofstream datafile;
    datafile.open(filename.c_str());
    //datafile << std::scientific;
    //datafile.precision(16);
    if(header.length() > 1)
        datafile << header << "\n";

    for(unsigned int i = 0; i < dataMatrix.size(); i++){
        for(unsigned int j = 0; j < dataMatrix[0].size(); j++){
            if(j<dataMatrix[0].size()-1){
                datafile << dataMatrix[i][j] << ", ";
            }
            else{
                datafile << dataMatrix[i][j] << "\n";
            }
        }
    }
    datafile.close();
}
