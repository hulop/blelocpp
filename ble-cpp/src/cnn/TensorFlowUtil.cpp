/*******************************************************************************
 * Copyright (c) 2018  IBM Corporation, Carnegie Mellon University and others
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

#include "TensorFlowUtil.hpp"
#include <fstream>
#include <sstream>
#include <string>

namespace {

    // Helper class used to load protobufs efficiently.
    class IfstreamInputStream : public ::google::protobuf::io::CopyingInputStream {
    public:
        explicit IfstreamInputStream(const std::string& file_name)
        : ifs_(file_name.c_str(), std::ios::in | std::ios::binary) {}
        ~IfstreamInputStream() { ifs_.close(); }
        
        int Read(void* buffer, int size) {
            if (!ifs_) {
                return -1;
            }
            ifs_.read(static_cast<char*>(buffer), size);
            return ifs_.gcount();
        }
        
    private:
        std::ifstream ifs_;
    };
    
    bool PortableReadFileToProto(const std::string& file_name,
                                 ::google::protobuf::MessageLite* proto) {
        ::google::protobuf::io::CopyingInputStreamAdaptor stream(
                                                                 new IfstreamInputStream(file_name));
        stream.SetOwnsCopyingStream(true);
        ::google::protobuf::io::CodedInputStream coded_stream(&stream);
        // Total bytes hard limit / warning limit are set to 1GB and 512MB
        // respectively.
        coded_stream.SetTotalBytesLimit(1024LL << 20, 512LL << 20);
        return proto->ParseFromCodedStream(&coded_stream);
    }
    
}

namespace loc{
    
    tensorflow::Status TensorFlowUtil::loadModelFromPath(const std::string& model_path,
                                                         std::unique_ptr<tensorflow::Session>* session) {
        assert(!model_path.empty());
        
        tensorflow::SessionOptions options;
        ////////
        // caution : set number of thread as one to prevent deadlock bug of tensorfow
        // feature release of tensorflow may solve this
        // https://github.com/tensorflow/tensorflow/issues/4917
        //
        tensorflow::ConfigProto& config = options.config;
        config.set_inter_op_parallelism_threads(1);
        ////////
        tensorflow::Session* session_pointer = nullptr;
        tensorflow::Status session_status = tensorflow::NewSession(options, &session_pointer);
        if (!session_status.ok()) {
            LOG(ERROR) << "Could not create TensorFlow Session: " << session_status;
            return session_status;
        }
        session->reset(session_pointer);
        
        tensorflow::GraphDef tensorflow_graph;
        const bool read_proto_succeeded = PortableReadFileToProto(model_path, &tensorflow_graph);
        if (!read_proto_succeeded) {
            LOG(ERROR) << "Failed to load model proto from" << model_path;
            return tensorflow::errors::NotFound(model_path);
        }
        
        tensorflow::Status create_status = (*session)->Create(tensorflow_graph);
        if (!create_status.ok()) {
            LOG(ERROR) << "Could not create TensorFlow Graph: " << create_status;
            return create_status;
        }
        
        return tensorflow::Status::OK();
    }

}
