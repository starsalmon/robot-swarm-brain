FROM rover-depth:jazzy
ENV PIP_BREAK_SYSTEM_PACKAGES=1
RUN echo numpy==1.26.4 > /c.txt && pip3 install --no-cache-dir -c /c.txt pillow timm einops mmengine scipy matplotlib
RUN pip3 install --no-cache-dir --no-deps -c /c.txt git+https://github.com/lpiccinelli-eth/UniDepth.git || echo UNIDEPTH_FAIL
RUN pip3 install --no-cache-dir wandb xformers --no-deps || true
RUN pip3 install --no-cache-dir --no-deps --force-reinstall torchvision --index-url https://download.pytorch.org/whl/cu126 && python3 -c "import torch,torchvision;print(torch.__version__,torchvision.__version__)"
RUN pip3 install --no-cache-dir -c /c.txt pydantic wandb gdown
