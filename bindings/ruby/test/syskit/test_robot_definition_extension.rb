# frozen_string_literal: true

require 'rock_gazebo/syskit/test'
require_relative '../helpers'

module RockGazebo
    module Syskit
        describe RobotDefinitionExtension do
            include Helpers

            before do
                @robot_model = ::Syskit::Robot::RobotDefinition.new
                Roby.app.using_task_library 'rock_gazebo'
                require 'models/orogen/rock_gazebo'
            end

            describe "#resolve_frame_element_from_full_name" do
                it "finds the link of a simple frame from a plugin context" do
                    xml = <<~XML
                        <model name="m">
                            <link name="root" />
                            <link name="other">
                                <plugin><frame>root</frame></plugin>
                            </link>
                        </model>
                    XML

                    xml = REXML::Document.new(xml).root
                    frame_element = xml.elements["//frame"]
                    assert_equal(
                        xml.elements["//link[@name=\"root\"]"],
                        RobotDefinitionExtension.resolve_frame_element_from_full_name(
                            frame_element, "root"
                        )
                    )
                end

                it "finds the model of a simple frame from a plugin context" do
                    xml = <<~XML
                        <model name="m">
                            <link name="root" />
                            <link name="other"><plugin><frame>m</frame></plugin></link>
                        </model>
                    XML

                    xml = REXML::Document.new(xml).root
                    frame_element = xml.elements["//frame"]
                    assert_equal(
                        xml,
                        RobotDefinitionExtension.resolve_frame_element_from_full_name(
                            frame_element, "m"
                        )
                    )
                end

                it "resolves a recursive name" do
                    xml = <<~XML
                        <model name="m">
                            <link name="root" />
                            <link name="other">
                                <plugin><frame>m::root</frame></plugin>
                            </link>
                        </model>
                    XML

                    xml = REXML::Document.new(xml).root
                    frame_element = xml.elements["//frame"]
                    assert_equal(
                        xml.elements["//link[@name=\"root\"]"],
                        RobotDefinitionExtension.resolve_frame_element_from_full_name(
                            frame_element, "m::root"
                        )
                    )
                end
            end

            describe '#find_actual_model' do
                before do
                    @robot_sdf =
                        ::SDF::Root.load('model://simple_model', flatten: false)
                                   .each_model.first
                    root = ::SDF::Root.load(
                        expand_fixture_world('attached_simple_model.world'),
                        flatten: false
                    )
                    @world = root.each_world.first
                end

                it 'resolves a toplevel model' do
                    actual_model, enclosing_model = @robot_model.find_actual_model(
                        'attachment', @world.each_model.to_a
                    )
                    assert_equal @world.each_model.first, actual_model
                    assert_nil enclosing_model
                end

                it 'resolves a model-in-model as well as its root' do
                    actual_model, enclosing_model = @robot_model.find_actual_model(
                        'included_model', @world.each_model.to_a
                    )
                    assert_equal @world.each_model.first.each_model.first, actual_model
                    assert_equal @world.each_model.first, enclosing_model
                end
            end

            describe '#define_submodel_device' do
                before do
                    root = ::SDF::Root.load(
                        expand_fixture_world('attached_simple_model.world'),
                        flatten: false
                    )
                    @world = root.each_world.first
                end

                it 'creates a device that exports the submodel\'s joints' do
                    attachment = @world.each_model.to_a.first
                    root_device = @robot_model.expose_gazebo_model(
                        attachment, 'gazebo_prefix'
                    )
                    device = @robot_model.define_submodel_device(
                        'included_model', root_device, attachment.each_model.first
                    )

                    assert_equal CommonModels::Devices::Gazebo::Model, device.model

                    submodel_driver_m = device.to_instance_requirements

                    driver_m = submodel_driver_m.to_component_model
                    assert_equal driver_m.included_model_joints_cmd_port,
                                 submodel_driver_m.joints_cmd_port.to_component_port
                end
            end

            describe 'root model' do
                before do
                    root = ::SDF::Root.load expand_fixture_world('simple_model.world')
                    @world = root.each_world.first
                    @robot_sdf = @world.each_model.first
                end

                describe 'the model export' do
                    before do
                        flexmock(Roby).should_receive(:warn_deprecated).at_least.once
                        @robot_model.load_gazebo(
                            @robot_sdf, 'gazebo', name: 'renamed_model'
                        )
                        @device = @robot_model.find_device('renamed_model')
                        @model_driver_m = @device.to_instance_requirements
                        @driver_m = @model_driver_m.to_component_model
                    end

                    it 'exposes the model device' do
                        assert_equal CommonModels::Devices::Gazebo::RootModel,
                                     @device.model
                    end

                    it 'sets up the deployment name' do
                        assert_equal ['gazebo::included_model'],
                                     @model_driver_m.deployment_hints.to_a
                    end

                    it 'sets up the transforms' do
                        assert_equal 'included_model', @device.frame_transform.from
                        assert_equal 'world', @device.frame_transform.to
                    end
                end

                describe 'deprecated link export behavior' do
                    before do
                        flexmock(Roby).should_receive(:warn_deprecated).at_least.once
                        @robot_model.load_gazebo(
                            @robot_sdf, 'gazebo', name: 'renamed_model'
                        )
                    end

                    it 'sets up the device transform on the link device' do
                        device = @robot_model.find_device('child_link')
                        assert_equal 'included_model::child', device.frame_transform.from
                        assert_equal 'world', device.frame_transform.to
                    end

                    it 'exposes the links from the model but does not prefix '\
                       'them with the model name' do
                        device = @robot_model.find_device('child_link')
                        link_driver_m = device.to_instance_requirements
                        driver_m = link_driver_m.to_component_model
                        assert_equal ['gazebo::included_model'],
                                     link_driver_m.deployment_hints.to_a
                        assert_equal(
                            driver_m.child_link_port,
                            link_driver_m.link_state_samples_port.to_component_port
                        )
                        transform = driver_m.find_transform_of_port(
                            driver_m.child_link_port
                        )
                        assert_equal 'child_source', transform.from
                        assert_equal 'child_target', transform.to

                        assert_equal 'included_model::child',
                                     link_driver_m.frame_mappings['child_source']
                    end
                end

                describe 'link export behavior' do
                    before do
                        @robot_model.load_gazebo(
                            @robot_sdf, 'gazebo',
                            name: 'renamed_model', prefix_device_with_name: true
                        )
                    end

                    it 'sets up the device transform on the link device' do
                        device = @robot_model.find_device('renamed_model_root_link')
                        assert_equal 'included_model::root', device.frame_transform.from
                        assert_equal 'world', device.frame_transform.to
                        device = @robot_model.find_device('renamed_model_child_link')
                        assert_equal 'included_model::child', device.frame_transform.from
                        assert_equal 'world', device.frame_transform.to
                    end

                    it 'exposes the links from the model' do
                        device, _, _, transform =
                            common_link_export_behavior

                        link_driver_m = device.to_instance_requirements
                        assert_equal ['gazebo::included_model'],
                                     link_driver_m.deployment_hints.to_a
                        assert_equal 'included_model::child',
                                     link_driver_m.frame_mappings['child_source']
                        assert_equal 'child_source', transform.from
                        assert_equal 'child_target', transform.to
                    end
                end
            end

            def common_link_export_behavior(link_name = 'child')
                device = @robot_model.find_device("renamed_model_#{link_name}_link")
                link_driver_m = device.to_instance_requirements
                driver_m = link_driver_m.to_component_model

                port = driver_m.find_port("renamed_model_#{link_name}_link")
                assert_equal port, link_driver_m.link_state_samples_port.to_component_port
                transform = driver_m.find_transform_of_port(port)
                [device, link_driver_m, driver_m, transform]
            end

            def common_sensor_export_behavior
                device = @robot_model.find_device('renamed_model_g_sensor')
                sensor_driver_m = device.to_instance_requirements
                driver_m = sensor_driver_m.to_component_model
                assert_equal OroGen.rock_gazebo.CameraTask, driver_m.model
                transform = driver_m.find_transform_of_port(driver_m.frame_port)
                [device, sensor_driver_m, driver_m, transform]
            end

            describe 'model with submodel' do
                before do
                    root = ::SDF::Root.load(
                        expand_fixture_world('attached_model_with_submodel.world'),
                        flatten: false
                    )
                    @world = root.each_world.first
                    @robot_sdf = @world.each_model.first.each_model.first
                    @robot_model.load_gazebo(
                        @robot_sdf, 'gazebo',
                        name: 'renamed_model',
                        prefix_device_with_name: true
                    )
                end
                it 'defines the enclosing device' do
                    assert @robot_model.find_device('attachment')
                end

                it 'sets up the device transform on the submodel device, '\
                   'using the submodel\'s root link as root frame' do
                    device = @robot_model.find_device('renamed_model')
                    assert_equal 'included_model::simple_model::root',
                                 device.frame_transform.from
                    assert_equal 'world', device.frame_transform.to
                end

                it 'defines a device that exposes the submodel' do
                    device = @robot_model.find_device('renamed_model')
                    submodel_driver_m = device.to_instance_requirements
                    assert_equal 'included_model::simple_model::root',
                                 submodel_driver_m.frame_mappings['renamed_model_source']
                end

                it "sets up the transforms on the submodel's sensors" do
                    device, = common_sensor_export_behavior
                    assert_equal 'included_model::simple_model::root',
                                 device.frame_transform.from
                    assert_equal 'world', device.frame_transform.to
                end

                it 'exposes the sensors from the submodel' do
                    _, sensor_driver_m, = common_sensor_export_behavior

                    assert_equal(
                        ['gazebo::attachment::included_model::simple_model::root::c::g_sensor'],
                        sensor_driver_m.deployment_hints.to_a
                    )
                end
            end

            describe 'model-in-model' do
                before do
                    root = ::SDF::Root.load(
                        expand_fixture_world('attached_simple_model.world'),
                        flatten: false
                    )
                    @world = root.each_world.first
                    @robot_sdf = @world.each_model.first.each_model.first
                    @robot_model.load_gazebo(
                        @robot_sdf, 'gazebo',
                        name: 'renamed_model',
                        prefix_device_with_name: true
                    )
                end

                it 'defines the enclosing device' do
                    assert @robot_model.find_device('attachment')
                end

                it 'sets up the device transform on the submodel device, '\
                   'using the submodel\'s root link as root frame' do
                    device = @robot_model.find_device('renamed_model')
                    assert_equal 'included_model::root', device.frame_transform.from
                    assert_equal 'world', device.frame_transform.to
                end

                it 'defines a device that exposes the submodel' do
                    device = @robot_model.find_device('renamed_model')
                    submodel_driver_m = device.to_instance_requirements
                    assert_equal 'included_model::root',
                                 submodel_driver_m.frame_mappings['renamed_model_source']
                end

                it 'ignores the links from the enclosing model' do
                    refute @robot_model.find_device('attachment_in_attachment_link')
                end

                it 'sets up the transforms on the submodel\'s links' do
                    device, = common_link_export_behavior
                    assert_equal "included_model::child", device.frame_transform.from
                    assert_equal "world", device.frame_transform.to
                end

                it 'provides the link names for the from and to of the link device' do
                    device, = common_link_export_behavior
                    assert_equal "included_model::child", device.sdf_from_link
                    assert_equal "world", device.sdf_to_link
                end

                it 'exposes the links from the submodel' do
                    _, link_driver_m, _, transform = common_link_export_behavior

                    assert_equal ['gazebo::attachment'],
                                 link_driver_m.deployment_hints.to_a
                    assert_equal(
                        'included_model::child',
                        link_driver_m.frame_mappings['included_model_child_source']
                    )
                    assert_equal 'included_model_child_source', transform.from
                    assert_equal 'included_model_child_target', transform.to
                end

                it "sets up the transforms on the submodel's sensors" do
                    device, = common_sensor_export_behavior
                    assert_equal 'included_model::root', device.frame_transform.from
                    assert_equal 'world', device.frame_transform.to
                end

                it 'exposes the sensors from the submodel' do
                    _, sensor_driver_m, = common_sensor_export_behavior

                    assert_equal(
                        ['gazebo::attachment::included_model::root::c::g_sensor'],
                        sensor_driver_m.deployment_hints.to_a
                    )
                end
            end

            describe 'model containing another model' do
                before do
                    root = ::SDF::Root.load(
                        expand_fixture_world('attached_simple_model.world'),
                        flatten: false
                    )
                    @world = root.each_world.first
                    @robot_sdf = @world.each_model.first
                    @robot_model.load_gazebo(
                        @robot_sdf, 'gazebo',
                        name: 'renamed_model', prefix_device_with_name: true
                    )
                end

                it 'sets up the transforms on the submodel\'s links' do
                    device, = common_link_export_behavior 'included_model_child'
                    assert_equal 'attachment::included_model::child',
                                 device.frame_transform.from
                    assert_equal 'world', device.frame_transform.to
                end

                it 'exposes the links from the submodel' do
                    _, link_driver_m, _, transform =
                        common_link_export_behavior 'included_model_child'

                    assert_equal ['gazebo::attachment'],
                                 link_driver_m.deployment_hints.to_a
                    assert_equal 'attachment::included_model::child',
                                 link_driver_m.frame_mappings['child_source']
                    assert_equal 'child_source', transform.from
                    assert_equal 'child_target', transform.to
                end

                it 'exposes the sensors from the submodel' do
                    device, sensor_driver_m, = common_sensor_export_behavior

                    assert_equal ['gazebo::attachment::included_model::root::c::g_sensor'],
                                 sensor_driver_m.deployment_hints.to_a
                    assert_equal 'attachment::included_model::root',
                                 device.frame_transform.from
                    assert_equal 'world',
                                 device.frame_transform.to
                end
            end

            describe '#sdf_export_link' do
                before do
                    @world = load_normalized_world("simple_model.world")
                    @robot_sdf = @world.each_model.first
                    @device = @robot_model.expose_gazebo_model(@robot_sdf, 'prefix')

                    @link_device = @robot_model.sdf_export_link(
                        @device,
                        as: 'some_links',
                        from_frame: 'prefix::root',
                        to_frame: 'prefix::child'
                    )
                end

                it 'creates a device with the relevant link_export dynamic service' do
                    plan = Roby::Plan.new
                    assert_equal 'prefix::root', @link_device.frame_transform.from
                    assert_equal 'prefix::child', @link_device.frame_transform.to
                    srv = @link_device.to_instance_requirements.instanciate(plan)
                    assert_equal 'some_links',
                                 srv.link_state_samples_port.to_actual_port.name
                end

                it "records all the exported links" do
                    assert_equal [@link_device], @robot_model.each_exported_link.to_a
                end

                it "exports the to and from frames as the sdf element of them" do
                    assert_equal "included_model::root",
                                 @link_device.sdf_from_link
                    assert_equal "included_model::child",
                                 @link_device.sdf_to_link
                end

                it "raises when the from frame is not a known link" do
                    model = @robot_model
                    dev = @device
                    assert_raises ArgumentError do
                        model.sdf_export_link(
                            dev,
                            as: "other_links",
                            from_frame: "banana::apple",
                            to_frame: "prefix::child"
                        )
                    end
                end

                it "raises when the to frame is not a known link" do
                    model = @robot_model
                    dev = @device
                    assert_raises ArgumentError do
                        model.sdf_export_link(
                            dev,
                            as: "other_links",
                            from_frame: "prefix::child",
                            to_frame: "banana::apple"
                        )
                    end
                end
            end

            describe '#sdf_export_joint' do
                before do
                    root = ::SDF::Root.load(
                        expand_fixture_world('simple_model.world'),
                        flatten: false
                    )
                    @world = root.each_world.first
                    @robot_sdf = @world.each_model.first
                    @device = @robot_model.expose_gazebo_model(@robot_sdf, 'prefix')

                    @joint_device = @robot_model.sdf_export_joint(
                        @device,
                        as: 'some_links', joint_names: ["included_model::root2child"]
                    )
                end

                it 'creates a device with the relevant joint_export dynamic service' do
                    plan = Roby::Plan.new
                    srv = @joint_device.to_instance_requirements.instanciate(plan)
                    assert_equal %w[included_model::root2child],
                                 srv.model.dynamic_service_options[:joint_names]
                end

                it 'sets ignore_joint_names to false by default' do
                    plan = Roby::Plan.new
                    srv = @joint_device.to_instance_requirements.instanciate(plan)
                    refute srv.model.dynamic_service_options[:ignore_joint_names]
                end

                it 'passes a ignore_joint_names flag' do
                    plan = Roby::Plan.new
                    joint_device = @robot_model.sdf_export_joint(
                        @device,
                        as: "other_links", joint_names: ["included_model::root2child"],
                        ignore_joint_names: true
                    )
                    srv = joint_device.to_instance_requirements.instanciate(plan)
                    assert srv.model.dynamic_service_options[:ignore_joint_names]
                end

                it 'sets position_offsets to empty by default' do
                    plan = Roby::Plan.new
                    srv = @joint_device.to_instance_requirements.instanciate(plan)
                    assert_equal [], srv.model.dynamic_service_options[:position_offsets]
                end

                it 'passes a position_offsets array' do
                    # NOTE: the dynamic service is expected to do the necessary
                    # validation
                    plan = Roby::Plan.new
                    joint_device = @robot_model.sdf_export_joint(
                        @device,
                        as: "other_links",
                        joint_names: ["included_model::root2child"],
                        ignore_joint_names: true,
                        position_offsets: [10]
                    )
                    srv = joint_device.to_instance_requirements.instanciate(plan)
                    assert srv.model.dynamic_service_options[:position_offsets]
                end

                it "records all the exported joints" do
                    assert_equal [@joint_device], @robot_model.each_exported_joint.to_a
                end

                it "raises when the given joint name is not a known joint" do
                    model = @robot_model
                    dev = @device
                    assert_raises ArgumentError do
                        model.sdf_export_joint(
                            dev,
                            as: "other_links",
                            joint_names: ["some::absurd::joint_name"],
                            ignore_joint_names: true,
                            position_offsets: [10]
                        )
                    end
                end
            end

            describe "CommonModels::Devices::Gazebo::RootModel" do
                before do
                    root = ::SDF::Root.load(
                        expand_fixture_world("attached_simple_model.world"),
                        flatten: false
                    )
                    @world = root.each_world.first
                    @robot_sdf = @world.each_model.first
                end

                describe "on the root model" do
                    before do
                        @robot_model.load_gazebo(
                            @robot_sdf, "gazebo",
                            name: "renamed_model", prefix_device_with_name: true
                        )

                        @device = @robot_model.find_device("renamed_model")
                        @link = @robot_model.sdf_export_link(
                            @device,
                            as: "some_links",
                            from_frame: "attachment::included_model::root",
                            to_frame: "attachment::included_model::child"
                        )
                        @joint = @robot_model.sdf_export_joint(
                            @device,
                            as: "some_joints",
                            joint_names: ["attachment::included_model::root2child"]
                        )
                    end

                    it "registers all exported links and joints in the root model" do
                        assert_equal [@joint],
                                     @device.gazebo_root_model.each_exported_joint.to_a
                        assert_equal [@link],
                                     @device.gazebo_root_model.each_exported_link.to_a
                    end

                    it "generates an fully instanciated model with all the exported " \
                       "joints and links" do
                        ir = @device.fully_instanciated_model
                        assert_equal [@device, @link, @joint], ir.arguments.values
                    end
                end

                describe "on a submodel" do
                    before do
                        submodel = @robot_sdf.each_model.first
                        @robot_model.load_gazebo(
                            submodel, "gazebo",
                            name: "renamed_model", prefix_device_with_name: true
                        )

                        @device = @robot_model.find_device("renamed_model")
                        @link = @robot_model.sdf_export_link(
                            @device,
                            as: "some_links", from_frame: "included_model::root",
                            to_frame: "included_model::child"
                        )
                        @joint = @robot_model.sdf_export_joint(
                            @device,
                            as: "some_joints", joint_names: ["included_model::root2child"]
                        )
                    end

                    it "registers all exported links and joints in the root model" do
                        root_model = @device.gazebo_root_model
                        assert_equal [@joint], root_model.each_exported_joint.to_a
                        assert_equal [@link], root_model.each_exported_link.to_a
                        assert_equal [@device], root_model.each_submodel.to_a
                    end

                    it "generates an fully instanciated model with all the exported " \
                       "joints and links" do
                        ir = @device.gazebo_root_model.fully_instanciated_model
                        assert_equal [@device.gazebo_root_model, @device, @link, @joint],
                                 ir.arguments.values
                    end
                end
            end

            def load_normalized_world(world)
                world = ::SDF::Root.load(expand_fixture_world(world), flatten: false)
                                   .each_world.first
                Rock::Gazebo.process_gazebo_world(world)
                world
            end
        end
    end
end
