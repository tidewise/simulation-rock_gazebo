# frozen_string_literal: true

module RockGazebo
    module Syskit
        # Gazebo-specific extensions to {Syskit::Robot::RobotDefinition}
        module RobotDefinitionExtension
            @plugin_device_mappings = {}
            def self.plugin_device_mappings
                @plugin_device_mappings ||= {}
            end

            # Registers a device model by a plugin task model
            def self.register_device_by_plugin_task_model(task_model, device_model)
                @plugin_device_mappings[task_model] = device_model
            end

            # Given a gazebo plugin, returns the device and device driver model that
            # should be used to handle it
            #
            # @param plugin [Plugin]
            # @param device_name [String] The desired device name
            # @param deployment_hint [String] The deployment hint for each of the tasks,
            #   this must be created using the using the desired `deployment_prefix`
            #   and the `relative_path` to the plugin.
            #
            # @return [nil,(Model<Syskit::Device>,Model<Syskit::Component>)]
            #   either nil if this type of sensor is not handled either by the
            #   rock-gazebo plugin or by the syskit integration (yet), or the
            #   device model and device driver that should be used for this
            #   sensor
            def plugin_task_to_device(
                root_model, task_element, task_model_name,
                name_scope, deployment_hint
            )
                task_model =
                    ::Syskit::TaskContext.find_model_from_orogen_name(task_model_name)
                # The task name is used to avoid ambiguity between multiple plugins
                # originated from the same plugin.
                task_name = task_element.attributes["name"]

                return unless (device_m = resolve_plugin_device_model(task_model))

                device_name = [
                    (name_scope unless name_scope.empty?),
                    task_name
                ].compact.join("_")
                device =
                    device(device_m, as: device_name, using: task_model)
                    .prefer_deployed_tasks("#{deployment_hint}#{task_name}")

                # Check if there is a frame or frame_from/frame_to attribute
                if (frame_name = task_element.elements["frame"]&.text)
                    device.frame(
                        RobotDefinitionExtension
                            .resolve_frame_full_name(root_model, task_element, frame_name)
                    )
                elsif (frame_from = task_element.elements["frame_from"]&.text)
                    frame_to = task_element.elements["frame_to"]&.text

                    from = RobotDefinitionExtension
                           .resolve_frame_full_name(root_model, task_element, frame_from)
                    to = RobotDefinitionExtension
                         .resolve_frame_full_name(root_model, task_element, frame_to)

                    device.frame_transform(from => to)
                end
                device
            end

            def self.resolve_frame_full_name(root_model, context, frame_name)
                return "world" if frame_name == "world"

                element = resolve_frame_element_from_full_name(context, frame_name)
                path = []
                while element != root_model.xml
                    path.unshift(element.attributes["name"])
                    element = element.parent
                end
                [root_model.name, *path].join("::")
            end

            def self.resolve_frame_element_from_full_name(context, frame_name)
                path = frame_name.split("::").freeze
                while context
                    queue = path.dup
                    frame_element = context
                    while frame_element
                        return frame_element if queue.empty?

                        next_name = queue.shift
                        xpath = "link[@name=\"#{next_name}\"] | " \
                                "model[@name=\"#{next_name}\"]"
                        frame_element = frame_element.get_elements(xpath).first
                    end

                    context = context.parent
                end

                raise ArgumentError, "cannot resolve frame name #{frame_name}"
            end

            def resolve_plugin_device_model(task_model)
                device_m = RobotDefinitionExtension.plugin_device_mappings[task_model]
                return device_m if device_m

                task_model.each_master_driver_service do |driver_m|
                    return driver_m.model
                end
                nil
            end

            # Registers a device as one of the exported joint devices
            def register_exported_joint_device(dev)
                @exported_joints ||= []
                @exported_joints << dev
            end

            # Iterate over exported joints
            def each_exported_joint
                return enum_for(__method__) unless block_given?

                @exported_joints&.each { yield _1 }
            end

            # Create a device that provides access to a subset of the joints
            #
            # Use it for instance, to get a simulated actuator
            def sdf_export_joint(
                model_dev,
                as: nil, joint_names: [],
                ignore_joint_names: false,
                position_offsets: [],
                control_modes: []
            )
                joint_sdfs = sdf_export_resolve_joint_sdf(model_dev, joint_names)
                joint_names = joint_sdfs.map do |el|
                    el.full_name(root: model_dev.gazebo_root_model.sdf.parent)
                end

                driver_def = model_dev.to_instance_requirements.to_component_model.dup
                driver_m = OroGen.rock_gazebo.ModelTask.specialize
                driver_srv = driver_m.require_dynamic_service(
                    'joint_export',
                    as: as, joint_names: joint_names,
                    position_offsets: position_offsets,
                    ignore_joint_names: ignore_joint_names,
                    control_modes: control_modes
                )

                driver_def.add_models([driver_m])
                driver_def.select_service(driver_srv)

                dev = device(CommonModels::Devices::Gazebo::Joint,
                             as: as, using: driver_def)
                register_exported_joint_device(dev)
                model_dev.gazebo_root_model.register_exported_joint(dev)
                dev
            end

            def sdf_export_resolve_joint_sdf(model_dev, joint_names)
                joint_names.map do |joint|
                    relative_joint_name = joint.split("::")[1..-1].join("::")
                    unless (sdf = model_dev.sdf.find_joint_by_name(relative_joint_name))
                        available_joints = model_dev.sdf.each_joint_with_name.map { _2 }
                        raise ArgumentError,
                              "cannot find joint #{relative_joint_name} inferred from " \
                              "frame #{joint}, within model #{model_dev.sdf.name}: " \
                              "available joints are #{available_joints.sort.join(", ")}"
                    end
                    sdf
                end
            end

            # Registers a device as one of the exported link devices
            def register_exported_link_device(dev)
                @exported_links ||= []
                @exported_links << dev
            end

            # Iterate over exported links
            def each_exported_link
                return enum_for(__method__) unless block_given?

                @exported_links&.each { yield _1 }
            end

            # Setup a link export feature of rock_gazebo::ModelTask
            #
            # @param [Syskit::Robot::MasterDeviceInstance] model_dev the model
            #   device that has the requested links
            # @param [String] as the name of the newly created device. It is
            #   also the name of the created port on the model task
            # @param [String] from_frame the 'from' frame of the exported link,
            #   which must match a link, model or frame name on the SDF model
            # @param [String] to_frame the 'to' frame of the exported link,
            #   which must match a link, model or frame name on the SDF model
            # @return [Syskit::Robot::MasterDeviceInstance] the exported link as
            #   a device instance of type CommonModels::Devices::Gazebo::Link
            def sdf_export_link(
                model_dev,
                as: nil, from_frame: nil, to_frame: nil,
                cov_position: nil, cov_orientation: nil,
                cov_velocity: nil
            )
                if !as
                    raise ArgumentError, 'provide a name for the device and port '\
                                         "through the 'as' option"
                elsif !from_frame
                    raise ArgumentError, "provide a name for the 'from' frame "\
                                         "through the 'from_frame' option"
                elsif !to_frame
                    raise ArgumentError, "provide a name for the 'to' frame "\
                                         "through the 'to_frame' option"
                end

                link_driver = model_dev.to_instance_requirements.to_component_model.dup
                link_driver_m = OroGen.rock_gazebo.ModelTask.specialize
                link_driver_srv = link_driver_m.require_dynamic_service(
                    'link_export',
                    as: as, frame_basename: as,
                    cov_position: cov_position,
                    cov_orientation: cov_orientation,
                    cov_velocity: cov_velocity
                )

                link_driver.add_models([link_driver_m])
                link_driver
                    .use_frames("#{as}_source" => from_frame,
                                "#{as}_target" => to_frame)
                    .select_service(link_driver_srv)

                dev = device(CommonModels::Devices::Gazebo::Link,
                             as: as, using: link_driver)
                from_link = sdf_export_resolve_link_sdf(model_dev, from_frame)
                from_link_frame = relative_link_name_from_root_model(model_dev, from_link)
                dev.sdf_from_link(from_link_frame)

                to_link = sdf_export_resolve_link_sdf(model_dev, to_frame)
                to_link_frame = relative_link_name_from_root_model(model_dev, to_link)
                dev.sdf_to_link(to_link_frame)

                dev.frame_transform(from_frame => to_frame) if from_frame != to_frame
                model_dev.gazebo_root_model.register_exported_link(dev)
                register_exported_link_device(dev)
                dev
            end

            def sdf_export_resolve_link_sdf(model_dev, frame_name)
                if frame_name == "world"
                    return resolve_enclosing_world(model_dev.sdf_model)
                end

                relative_link_name = frame_name.split("::")[1..-1].join("::")
                unless (sdf = model_dev.sdf.find_link_by_name(relative_link_name))
                    raise ArgumentError,
                          "cannot find link #{relative_link_name} inferred from frame " \
                          "#{frame_name}, within model #{model_dev.sdf.name}"
                end
                sdf
            end

            def relative_link_name_from_root_model(model_dev, link)
                root_model_parent = model_dev.gazebo_root_model.sdf.parent
                return "world" if link.instance_of?(::SDF::World)

                link.full_name(root: root_model_parent) || link.full_name
            end

            def find_actual_model(name, candidates)
                candidates = candidates.dup
                until candidates.empty?
                    m, root = candidates.shift
                    return m, root if m.name == name

                    children = m.each_model.map { |child_m| [child_m, root || m] }
                    candidates.concat(children)
                end
                nil
            end

            def create_frame_mappings_for_used_model(model)
                @link_frame_names ||= {}
                @link_frame_names[model] = model.name
                model.each_link_with_name do |l, _|
                    @link_frame_names[l] = "#{model.name}::#{l.full_name(root: model)}"
                end
            end

            def link_frame_name(sdf)
                @link_frame_names ||= {}
                @link_frame_names[sdf] ||= sdf.full_name(
                    root: resolve_enclosing_world(sdf)
                )
            end

            def define_submodel_device(name, enclosing_device, actual_sdf_model)
                normalized_name = normalize_name(name)
                submodel_driver_m = OroGen.rock_gazebo.ModelTask.specialize
                driver_srv = submodel_driver_m.require_dynamic_service(
                    'submodel_export',
                    as: normalized_name, frame_basename: normalized_name
                )

                unless (root_link = actual_sdf_model.each_link.first)
                    raise ArgumentError, 'cannot refer to a submodel that has no links'
                end

                link_frame = link_frame_name(root_link)

                submodel_driver_m =
                    submodel_driver_m
                    .to_instance_requirements
                    .prefer_deployed_tasks(*enclosing_device
                                           .to_instance_requirements
                                           .deployment_hints)
                    .with_arguments(model_dev: enclosing_device)
                    .use_frames("#{normalized_name}_source" => link_frame,
                                "#{normalized_name}_target" => 'world')
                    .select_service(driver_srv)
                submodel =
                    device(CommonModels::Devices::Gazebo::Model,
                           as: normalized_name, using: submodel_driver_m)
                    .doc("Gazebo: model #{name} inside #{enclosing_device.sdf.full_name}")
                    .frame_transform(link_frame => "world")
                    .sdf(actual_sdf_model)
                    .advanced
                enclosing_device.register_submodel(submodel)
                submodel
            end

            # @api private
            #
            # Enumerate the models that are exported via a rock_gazebo::ModelTask
            #
            # The export can happen directly by adding the task as a child of the model
            # tag, or indirecty because the model task has a gz attribute
            def resolve_exported_models(context)
                exported_models = []
                each_model_task_recursive(context) do |parent, xml|
                    if (exported_model_name = xml.attributes["gz"])
                        exported_models <<
                            resolve_model_from_name(parent, exported_model_name)
                    else
                        exported_models << parent
                    end
                end
                exported_models
            end

            # @api private
            #
            # Resolve `name` as a submodel (possibly recursively) from the given context
            def resolve_model_from_name(context, name)
                path = name.split("::")
                resolved = path.inject(context) do |sdf, single|
                    sdf&.find_model_by_name(single)
                end

                unless resolved
                    raise ArgumentError, "cannot resolve #{name} from #{context}"
                end
                resolved
            end

            # Recursively resolve the <task...> tags whose model is a
            # rock_gazebo::ModelTask
            #
            # @yieldparam [SDF::Model] model the model the task is defined in
            # @yieldparam [REXML::Element] task_xml the XML element of the task
            def each_model_task_recursive(context, &block)
                return enum_for(:each_model_task_recursive) unless block_given?

                context.each_direct_plugin do |plugin|
                    plugin.xml
                          .get_elements("task[@model=\"rock_gazebo::ModelTask\"]")
                          .each { |xml| yield(context, xml) }
                end

                context.each_direct_model do |model|
                    each_model_task_recursive(model, &block)
                end
            end

            # @api private
            #
            # Find the toplevel model that contains the given one
            #
            # @return [Model] the enclosing model. Might be the original model
            #   if it is toplevel.
            def resolve_enclosing_model(context)
                world = resolve_enclosing_world(context)
                exported_models = resolve_exported_models(world)

                while context.kind_of?(::SDF::Model)
                    return context if exported_models.include?(context)

                    context = context.parent
                end
                nil
            end

            # @api private
            #
            # Find the world that contains the given model
            #
            # @return [World,nil] the enclosing world, or nil if the model is
            #   not included in one
            def resolve_enclosing_world(node)
                node = node.parent while node && !node.kind_of?(::SDF::World)
                node
            end

            # Create device information that models how the rock-gazebo plugin
            # will handle this SDF model
            #
            # I.e. it creates devices that match the tasks the rock-gazebo
            # plugin will create when given this SDF information
            #
            # It does it for all the models in 'world', and for links and
            # plugins only for 'model'
            #
            # @param [SDF::Model] robot_model the SDF model for this robot
            # @param [String] name the name of the model in the world, if it
            #   differs from the robot_model name (e.g. if you have multiple
            #   instances of the same robot)
            # @param [Array<SDF::Model>] models a set of models that should be
            #   exposed as devices on this robot model. Note that plugins and
            #   links are only exposed for the robot_model. It must contain
            #   robot_model
            # @param [Boolean] prefix_device_with_name if true, the name of
            #   the created devices are prefixed with the 'name' argument and an
            #   underscore. This will become the new default, and using false
            #   generates a deprecation warning. Setting it to true ensures that
            #   the device names stay the same regardless of the SDF world it's
            #   built in.
            # @return [Syskit::Robot::Device] the device that represents the model
            # @raise [ArgumentError] if models does not contain robot_model
            def load_gazebo(
                model, deployment_prefix,
                name: model.name, reuse: nil,
                prefix_device_with_name: Syskit.prefix_device_with_name
            )
                # Allow passing a profile instead of a robot definition
                reuse = reuse.robot if reuse.respond_to?(:robot)
                enclosing_model = resolve_enclosing_model(model)
                unless enclosing_model
                    raise "found expected model, but there are no ModelTask exporting it"
                end

                unless prefix_device_with_name
                    Roby.warn_deprecated <<~EOMSG
                        The link naming scheme in #use_sdf_model and #use_gazebo_model
                        will change from using the link name as device name to
                        prefixing it with the name given to #use_sdf_model
                        (which defaults to the model name itself) set the
                        prefix_device_with_name: option to true to enable the
                        new behavior and remove this warning This warning will
                        become an error before the functionality completely
                        disappears
                    EOMSG
                end

                deployment_prefix += "::"
                if enclosing_model != model
                    create_frame_mappings_for_used_model(model)
                    enclosing_device = expose_gazebo_model(
                        enclosing_model, deployment_prefix,
                        reuse: reuse, device_name: enclosing_model.name
                    )
                    model_device = define_submodel_device(name, enclosing_device, model)
                    prefix_device_with_name = true
                    deployment_prefix +=
                        "#{sdf_relative_path(enclosing_model.parent, model)}::"
                else
                    enclosing_device = expose_gazebo_model(
                        enclosing_model, deployment_prefix,
                        reuse: reuse, device_name: name
                    )
                    model_device = enclosing_device
                    enclosing_device.advanced = false
                    deployment_prefix += model.name + "::"
                end

                load_gazebo_robot_model(
                    model, enclosing_device,
                    name: name, reuse: reuse,
                    prefix_device_with_name: prefix_device_with_name,
                    deployment_prefix: deployment_prefix
                )
                model_device
            end

            # @api private
            #
            # Define devices for each model in the world
            #
            # @param [Array<SDF::Model>] models the SDF representation of the models
            def expose_gazebo_model(
                sdf, deployment_prefix,
                reuse: nil, device_name: normalize_name(sdf.name)
            )
                if reuse && (existing = reuse.find_device(device_name))
                    register_device(device_name, existing)
                    return existing
                end

                device(CommonModels::Devices::Gazebo::RootModel,
                       as: device_name,
                       using: OroGen.rock_gazebo.ModelTask)
                    .prefer_deployed_tasks(
                        "#{deployment_prefix}#{normalize_name(sdf.name)}"
                    )
                    .frame_transform(link_frame_name(sdf) => 'world')
                    .advanced
                    .sdf(sdf)
                    .doc("Gazebo: the #{sdf.name} model")
            end

            # @api private
            #
            # Normalize a Gazebo name to use in Syskit models
            def normalize_name(name)
                name.gsub(/::+/, '_')
            end

            # @api private
            #
            # Describes recursively all plugins in the model and its submodels
            def load_gazebo_robot_submodels(
                root_model, sdf_model,
                name: sdf_model.name,
                prefix_device_with_name:,
                deployment_prefix: ""
            )
                plugins =
                    if Syskit.scope_device_name_with_links_and_submodels
                        sdf_model.each_direct_plugin
                    else
                        sdf_model.each_plugin
                    end

                models =
                    if Syskit.scope_device_name_with_links_and_submodels
                        sdf_model.each_direct_model
                    else
                        sdf_model.each_model
                    end

                sensors =
                    if Syskit.scope_device_name_with_links_and_submodels
                        sdf_model.each_direct_sensor
                    else
                        sdf_model.each_sensor
                    end

                sensors.each do |sensor|
                    sensor.each_plugin do |plugin|
                        gazebo_define_plugin_devices(
                            root_model, sdf_model, plugin,
                            model_name: name,
                            prefix_device_with_name: prefix_device_with_name,
                            deployment_prefix: deployment_prefix
                        )
                    end
                end

                plugins.each do |plugin|
                    gazebo_define_plugin_devices(
                        root_model, sdf_model, plugin,
                        model_name: name,
                        prefix_device_with_name: prefix_device_with_name,
                        deployment_prefix: deployment_prefix
                    )
                end

                models.each do |model|
                    load_gazebo_robot_submodels(
                        root_model, model,
                        name: name + "_" + model.name,
                        prefix_device_with_name: true,
                        deployment_prefix: deployment_prefix + model.name + "::"
                    )
                end
            end

            # @api private
            #
            # Define devices for all links and plugins in the model and its submodels
            #
            # @param [SDF::Model] sdf_model SDF model of this profile's robot,
            #   as included in the current gazebo world
            def load_gazebo_robot_model(
                sdf_model, root_device,
                reuse: nil, name: sdf_model.name,
                prefix_device_with_name:,
                deployment_prefix: ""
            )
                if (prefix = sdf_model.full_name(root: root_device.sdf))
                    frame_prefix = "#{normalize_name(prefix)}_"
                end

                if Syskit.use_gazebo_automatic_links
                    sdf_model.each_link_with_name do |l, l_name|
                        gazebo_define_link_device(
                            root_device, sdf_model, l, l_name,
                            model_name: name, reuse: reuse, frame_prefix: frame_prefix,
                            prefix_device_with_name: prefix_device_with_name
                        )
                    end
                end

                load_gazebo_robot_submodels(
                    sdf_model, sdf_model,
                    name: name, prefix_device_with_name: prefix_device_with_name,
                    deployment_prefix: deployment_prefix
                )
            end

            def gazebo_define_link_device(
                root_device, sdf_model, link, link_name,
                model_name: sdf_model.name, frame_prefix: '',
                prefix_device_with_name:, reuse: false
            )
                device_name = "#{normalize_name(link_name)}_link"
                if prefix_device_with_name
                    device_name = "#{normalize_name(model_name)}_#{device_name}"
                end
                if reuse && (existing = reuse.find_device(device_name))
                    register_device(device_name, existing)
                    return
                end
                frame_basename = "#{frame_prefix}#{normalize_name(link.name)}"

                link_frame = link_frame_name(link)

                link_driver_m = OroGen.rock_gazebo.ModelTask.specialize
                driver_srv = link_driver_m.require_dynamic_service(
                    'link_export', as: device_name, frame_basename: frame_basename
                )
                link_driver_m =
                    link_driver_m
                    .to_instance_requirements
                    .prefer_deployed_tasks(
                        *root_device.to_instance_requirements.deployment_hints
                    )
                    .with_arguments(model_dev: root_device)
                    .use_frames("#{frame_basename}_source" => link_frame,
                                "#{frame_basename}_target" => 'world')
                    .select_service(driver_srv)
                dev =
                    device(
                        CommonModels::Devices::Gazebo::Link,
                        as: device_name, using: link_driver_m
                    )
                    .doc("Gazebo: state of the #{link.name} link of #{sdf_model.name}")
                    .frame_transform(link_frame => 'world')
                    .advanced

                link_full_name = link.full_name(root: root_device.sdf)
                dev.sdf_from_link(link_full_name)
                dev.sdf_to_link("world")
                dev
            end

            def gazebo_define_plugin_devices(
                root_model, sdf_model, plugin,
                model_name: sdf_model.name, prefix_device_with_name:,
                deployment_prefix: ""
            )
                name_scope = nil
                model_path = sdf_relative_path(sdf_model, plugin.parent)
                if prefix_device_with_name
                    if Syskit.scope_device_name_with_links_and_submodels
                        name_scope = normalize_name(model_path) unless model_path.empty?
                    end

                    name_scope = [
                        (normalize_name(model_name) if model_name),
                        name_scope
                    ].compact.join("_")
                end

                deployment_hint = "#{deployment_prefix}#{model_path}"
                plugin.xml.elements.to_a("task").map do |task_element|
                    task_model_name = task_element.attributes["model"]
                    # ModelTask is supported directly, ignore here
                    next if task_model_name == "rock_gazebo::ModelTask"

                    device = plugin_task_to_device(
                        root_model, task_element, task_model_name, name_scope, deployment_hint
                    )
                    unless device
                        RockGazebo.warn(
                            "Robot#load_gazebo: no device model associated with " \
                            "#{task_model_name}"
                        )
                        next
                    end

                    device.doc "Gazebo: plugin of #{sdf_model.full_name}"
                    device.sdf(task_element)
                end
            end

            def sdf_relative_path(from, to)
                elements = []
                current = to
                while current != from
                    elements.unshift current
                    current = current.parent
                end
                elements.map(&:name).join("::")
            end
        end
        ::Syskit::Robot::RobotDefinition.include RobotDefinitionExtension
    end
end
