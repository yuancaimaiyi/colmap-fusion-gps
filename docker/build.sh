docker build -t="VisMap:latest" .;
docker run --gpus all -w /working -v $1:/working -it VisMap:latest;
