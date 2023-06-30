FROM ubuntu:latest

RUN apt update
RUN apt install git -y
RUN apt install python3 python3-pip -y
RUN apt install curl nano -y
