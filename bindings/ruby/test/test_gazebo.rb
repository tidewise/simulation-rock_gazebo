# frozen_string_literal: true

require "orogen"
require "rock_gazebo/test"
require "rock/gazebo"
require "rock/bundles"
require "minitest/autorun"

module Rock
    describe Gazebo do
        describe ".process_sdf_world" do
            it "resolve the 'filename' argument of a <task ...> element "\
               "in <plugin ...>" do
                loader = flexmock
                task_model = create_task_model_mock(loader, "some::Task")
                sdf = ::SDF::Root.from_string(
                    <<~SDF
                    <sdf><world><plugin>
                        <task model="some::Task" />
                    </plugin></world></sdf>
                    SDF
                )

                xml = Gazebo.process_sdf_world(sdf, loader: loader)
                task = REXML::XPath.first(xml, "//task")
                assert_equal "some::Task_project_path", task.attributes["filename"]
            end

            it "auto-loads a task's dependent typekits" do
                loader = flexmock
                tk_a = create_typekit_mock(loader, "tk_a")
                tk_b = create_typekit_mock(loader, "tk_b")
                task_model = create_task_model_mock(
                    loader, "some::Task",
                    interface_typekits: { "tk_a" => tk_a, "tk_b" => tk_b }
                )

                sdf = ::SDF::Root.from_string(
                    <<~SDF
                    <sdf><world>
                    <plugin name="rock::gz::PluginTask">
                        <task model="some::Task" />
                    </plugin></world></sdf>
                    SDF
                )

                xml = Gazebo.process_sdf_world(sdf, loader: loader)
                expected = %w[
                    tk_a_library_path tk_a_transport_corba_path
                    tk_a_transport_mqueue_path tk_a_transport_typelib_path
                    tk_b_library_path tk_b_transport_corba_path
                    tk_b_transport_mqueue_path tk_b_transport_typelib_path
                ]
                assert_world_loads_libraries(xml, expected)
            end

            def assert_world_loads_libraries(xml, expected)
                loads =
                    REXML::XPath
                    .each(xml, "/sdf/world/plugin[@name='rock_gazebo::PluginTask']/load")
                    .map { |xml| xml.attributes["path"] }
                assert_equal expected.to_set, loads.to_set,
                    "expected #{xml.to_s} to have load statements"
            end

            def create_task_model_mock(
                loader, name,
                interface_typekits: {}, project_name: "#{name}_project"
            )
                mock = flexmock
                mock.should_receive(project: Struct.new(:name).new(project_name))
                loader.should_receive(:task_model_from_name)
                      .with(name).and_return(mock)
                loader.should_receive(:task_library_path_from_name)
                      .with(project_name).and_return("#{project_name}_path")

                interface_types =
                    interface_typekits.each_key.map { |tk_name| "#{tk_name}_type" }
                mock.should_receive(:each_interface_type).and_iterates(*interface_types)
                interface_typekits.each do |typekit_name, typekit|
                    loader.should_receive(:typekit_for).with("#{typekit_name}_type")
                          .and_return(typekit)
                end
            end

            def create_typekit_mock(loader, name, virtual: false)
                mock = flexmock
                mock.should_receive(name: name, virtual?: virtual)
                loader.should_receive(:typekit_library_path_from_name)
                      .with(name).and_return("#{name}_library_path")
                %w[typelib mqueue corba].each do |transport|
                    loader.should_receive(:transport_library_path_from_name)
                          .with(name, transport)
                          .and_return("#{name}_transport_#{transport}_path")
                end
                mock
            end
        end
        describe ".prepare_spawn" do
            it "preprocesses .world.erb files and passes a generated tempfile" do
                file = Tempfile.new(["test", ".world.erb"])
                file.write("<sdf version='1.6'><world name='default'><plugin name='p' filename='f'/></world></sdf>")
                file.flush

                spawn_args = nil
                Gazebo.prepare_spawn("gz", "sim", file.path) do |args|
                    spawn_args = args
                end

                refute_equal file.path, spawn_args.last
                assert spawn_args.last.end_with?(".world")
                assert File.exist?(spawn_args.last)
                content = File.read(spawn_args.last)
                assert_match(/<world name='default'>/, content)
            ensure
                file&.close!
            end
        end
    end
end
