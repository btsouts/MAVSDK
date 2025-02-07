#include <future>
#include <string>

#include "core/core.grpc.pb.h"
#include "mavsdk.h"

namespace mavsdk {
namespace backend {

template<typename Mavsdk = Mavsdk>
class CoreServiceImpl final : public mavsdk::rpc::core::CoreService::Service {
public:
    CoreServiceImpl(Mavsdk &dc) :
        _dc(dc),
        _stop_promise(std::promise<void>()),
        _stop_future(_stop_promise.get_future())
    {}

    grpc::Status SubscribeConnectionState(
        grpc::ServerContext * /* context */,
        const rpc::core::SubscribeConnectionStateRequest * /* request */,
        grpc::ServerWriter<rpc::core::ConnectionStateResponse> *writer) override
    {
        _dc.register_on_discover([&writer](const uint64_t uuid) {
            const auto rpc_connection_state_response = createRpcConnectionStateResponse(uuid, true);
            writer->Write(rpc_connection_state_response);
        });

        _dc.register_on_timeout([&writer](const uint64_t uuid) {
            const auto rpc_connection_state_response =
                createRpcConnectionStateResponse(uuid, false);
            writer->Write(rpc_connection_state_response);
        });

        _stop_future.wait();
        return grpc::Status::OK;
    }

    // For now, the running plugins are hardcoded and we assume they are always started by the
    // backend.
    grpc::Status
    ListRunningPlugins(grpc::ServerContext * /* context */,
                       const rpc::core::ListRunningPluginsRequest * /* request */,
                       mavsdk::rpc::core::ListRunningPluginsResponse *response) override
    {
        std::string plugin_names[10] = {"action",
                                        "calibration",
                                        "camera",
                                        "core",
                                        "gimbal",
                                        "info",
                                        "mission",
                                        "offboard",
                                        "param",
                                        "telemetry"};

        for (const auto plugin_name : plugin_names) {
            auto plugin_info = response->add_plugin_info();
            plugin_info->set_name(plugin_name);
            plugin_info->set_address("localhost");
            plugin_info->set_port(50051);
        }

        return grpc::Status::OK;
    }

    void stop() { _stop_promise.set_value(); }

private:
    Mavsdk &_dc;
    std::promise<void> _stop_promise;

    std::future<void> _stop_future;

    static mavsdk::rpc::core::ConnectionStateResponse
    createRpcConnectionStateResponse(const uint64_t uuid, const bool is_connected)
    {
        auto rpc_connection_state = new rpc::core::ConnectionState();
        rpc_connection_state->set_uuid(uuid);
        rpc_connection_state->set_is_connected(is_connected);

        mavsdk::rpc::core::ConnectionStateResponse rpc_connection_state_response;
        rpc_connection_state_response.set_allocated_connection_state(rpc_connection_state);

        return rpc_connection_state_response;
    }
};

} // namespace backend
} // namespace mavsdk
