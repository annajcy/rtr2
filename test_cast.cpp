#include <iostream>

class IRenderPipeline {
public:
    virtual ~IRenderPipeline() = default;
};

class IFramePreparePipeline {
public:
    virtual ~IFramePreparePipeline() = default;
    virtual void prepare_frame() = 0;
};

class ForwardEditorPipeline : public IRenderPipeline, public IFramePreparePipeline {
public:
    void prepare_frame() override { std::cout << "prepared\n"; }
};

void run(IRenderPipeline* p) {
    if (auto* fpp = dynamic_cast<IFramePreparePipeline*>(p)) {
        fpp->prepare_frame();
    } else {
        std::cout << "dynamic_cast failed\n";
    }
}

int main() {
    ForwardEditorPipeline fep;
    run(&fep);
    return 0;
}
