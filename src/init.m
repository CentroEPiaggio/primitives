disp('Cerca la cartella tbxmanager in cui sono installati i files della libreria MPT e sostituisci il path con quello')
cd ../tbxmanager
startup
cd ../src

addpath(pwd) % add current dir to path
addpath('cprnd/');
