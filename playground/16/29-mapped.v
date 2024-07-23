// Benchmark "adder" written by ABC on Wed Jul 17 20:24:49 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n185, new_n186, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n308, new_n311, new_n313,
    new_n314, new_n316;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n24x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  and002aa1n02x5               g002(.a(\b[3] ), .b(\a[4] ), .o(new_n98));
  inv040aa1d32x5               g003(.a(\a[3] ), .o1(new_n99));
  inv040aa1d32x5               g004(.a(\b[2] ), .o1(new_n100));
  nand42aa1n06x5               g005(.a(new_n100), .b(new_n99), .o1(new_n101));
  nand42aa1n06x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nand02aa1d04x5               g007(.a(new_n101), .b(new_n102), .o1(new_n103));
  nor042aa1n12x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  nand42aa1n20x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nand42aa1d28x5               g010(.a(\b[0] ), .b(\a[1] ), .o1(new_n106));
  aoi012aa1d24x5               g011(.a(new_n104), .b(new_n105), .c(new_n106), .o1(new_n107));
  inv000aa1d42x5               g012(.a(\a[4] ), .o1(new_n108));
  aboi22aa1n06x5               g013(.a(\b[3] ), .b(new_n108), .c(new_n99), .d(new_n100), .out0(new_n109));
  oaoi13aa1n12x5               g014(.a(new_n98), .b(new_n109), .c(new_n107), .d(new_n103), .o1(new_n110));
  nor022aa1n16x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n09x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nanp02aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nor022aa1n08x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1d18x5               g019(.a(new_n113), .b(new_n112), .c(new_n114), .d(new_n111), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .out0(new_n116));
  tech160nm_fixnrc02aa1n04x5   g021(.a(\b[4] ), .b(\a[5] ), .out0(new_n117));
  nor043aa1n06x5               g022(.a(new_n115), .b(new_n116), .c(new_n117), .o1(new_n118));
  and002aa1n02x7               g023(.a(\b[5] ), .b(\a[6] ), .o(new_n119));
  oai012aa1n02x7               g024(.a(new_n112), .b(new_n114), .c(new_n111), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\a[5] ), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\a[6] ), .o1(new_n122));
  inv040aa1d32x5               g027(.a(\b[4] ), .o1(new_n123));
  aboi22aa1n09x5               g028(.a(\b[5] ), .b(new_n122), .c(new_n121), .d(new_n123), .out0(new_n124));
  oai013aa1n06x5               g029(.a(new_n120), .b(new_n115), .c(new_n119), .d(new_n124), .o1(new_n125));
  xorc02aa1n12x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n06x5               g031(.a(new_n126), .b(new_n125), .c(new_n110), .d(new_n118), .o1(new_n127));
  xorc02aa1n12x5               g032(.a(\a[10] ), .b(\b[9] ), .out0(new_n128));
  xnbna2aa1n03x5               g033(.a(new_n128), .b(new_n127), .c(new_n97), .out0(\s[10] ));
  aobi12aa1n06x5               g034(.a(new_n128), .b(new_n127), .c(new_n97), .out0(new_n130));
  nor042aa1d18x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand42aa1d28x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  oaoi03aa1n12x5               g039(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n135));
  oab012aa1n02x4               g040(.a(new_n134), .b(new_n130), .c(new_n135), .out0(new_n136));
  norp03aa1n02x5               g041(.a(new_n130), .b(new_n133), .c(new_n135), .o1(new_n137));
  norp02aa1n02x5               g042(.a(new_n136), .b(new_n137), .o1(\s[11] ));
  inv000aa1d42x5               g043(.a(new_n131), .o1(new_n139));
  nor042aa1n06x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand42aa1n16x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nanb02aa1n02x5               g046(.a(new_n140), .b(new_n141), .out0(new_n142));
  nano22aa1n02x4               g047(.a(new_n136), .b(new_n139), .c(new_n142), .out0(new_n143));
  oai012aa1n02x5               g048(.a(new_n133), .b(new_n130), .c(new_n135), .o1(new_n144));
  aoi012aa1n02x5               g049(.a(new_n142), .b(new_n144), .c(new_n139), .o1(new_n145));
  norp02aa1n02x5               g050(.a(new_n143), .b(new_n145), .o1(\s[12] ));
  nano23aa1d15x5               g051(.a(new_n131), .b(new_n140), .c(new_n141), .d(new_n132), .out0(new_n147));
  nand23aa1d12x5               g052(.a(new_n147), .b(new_n126), .c(new_n128), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  aoai13aa1n04x5               g054(.a(new_n149), .b(new_n125), .c(new_n110), .d(new_n118), .o1(new_n150));
  aoi112aa1n02x5               g055(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n151));
  aoi112aa1n03x5               g056(.a(new_n151), .b(new_n140), .c(new_n147), .d(new_n135), .o1(new_n152));
  xnrc02aa1n12x5               g057(.a(\b[12] ), .b(\a[13] ), .out0(new_n153));
  xobna2aa1n03x5               g058(.a(new_n153), .b(new_n150), .c(new_n152), .out0(\s[13] ));
  orn002aa1n03x5               g059(.a(\a[13] ), .b(\b[12] ), .o(new_n155));
  aoai13aa1n02x5               g060(.a(new_n155), .b(new_n153), .c(new_n150), .d(new_n152), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n02x5               g062(.a(\b[13] ), .b(\a[14] ), .out0(new_n158));
  norp02aa1n02x5               g063(.a(new_n158), .b(new_n153), .o1(new_n159));
  inv000aa1n02x5               g064(.a(new_n159), .o1(new_n160));
  oao003aa1n03x5               g065(.a(\a[14] ), .b(\b[13] ), .c(new_n155), .carry(new_n161));
  aoai13aa1n04x5               g066(.a(new_n161), .b(new_n160), .c(new_n150), .d(new_n152), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor022aa1n08x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nand02aa1n06x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nor022aa1n08x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  nanp02aa1n04x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  aoi112aa1n02x5               g073(.a(new_n168), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n169));
  aoai13aa1n02x5               g074(.a(new_n168), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(\s[16] ));
  nano23aa1n02x5               g076(.a(new_n164), .b(new_n166), .c(new_n167), .d(new_n165), .out0(new_n172));
  nona22aa1n03x5               g077(.a(new_n172), .b(new_n158), .c(new_n153), .out0(new_n173));
  nor042aa1n06x5               g078(.a(new_n173), .b(new_n148), .o1(new_n174));
  aoai13aa1n12x5               g079(.a(new_n174), .b(new_n125), .c(new_n110), .d(new_n118), .o1(new_n175));
  nanp02aa1n03x5               g080(.a(new_n147), .b(new_n135), .o1(new_n176));
  nona22aa1n02x4               g081(.a(new_n176), .b(new_n151), .c(new_n140), .out0(new_n177));
  nona23aa1n09x5               g082(.a(new_n167), .b(new_n165), .c(new_n164), .d(new_n166), .out0(new_n178));
  nor003aa1n02x5               g083(.a(new_n178), .b(new_n158), .c(new_n153), .o1(new_n179));
  aoi012aa1n02x5               g084(.a(new_n166), .b(new_n164), .c(new_n167), .o1(new_n180));
  oai012aa1n02x7               g085(.a(new_n180), .b(new_n161), .c(new_n178), .o1(new_n181));
  aoi012aa1n09x5               g086(.a(new_n181), .b(new_n177), .c(new_n179), .o1(new_n182));
  xorc02aa1n02x5               g087(.a(\a[17] ), .b(\b[16] ), .out0(new_n183));
  xnbna2aa1n03x5               g088(.a(new_n183), .b(new_n175), .c(new_n182), .out0(\s[17] ));
  inv000aa1d42x5               g089(.a(\a[17] ), .o1(new_n185));
  inv000aa1d42x5               g090(.a(\b[16] ), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(new_n186), .b(new_n185), .o1(new_n187));
  inv000aa1n02x5               g092(.a(new_n115), .o1(new_n188));
  norp02aa1n02x5               g093(.a(new_n124), .b(new_n119), .o1(new_n189));
  aobi12aa1n06x5               g094(.a(new_n120), .b(new_n188), .c(new_n189), .out0(new_n190));
  aob012aa1n06x5               g095(.a(new_n190), .b(new_n110), .c(new_n118), .out0(new_n191));
  oabi12aa1n02x7               g096(.a(new_n181), .b(new_n152), .c(new_n173), .out0(new_n192));
  aoai13aa1n02x5               g097(.a(new_n183), .b(new_n192), .c(new_n191), .d(new_n174), .o1(new_n193));
  nor002aa1d32x5               g098(.a(\b[17] ), .b(\a[18] ), .o1(new_n194));
  nand42aa1n08x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  nanb02aa1n12x5               g100(.a(new_n194), .b(new_n195), .out0(new_n196));
  xobna2aa1n03x5               g101(.a(new_n196), .b(new_n193), .c(new_n187), .out0(\s[18] ));
  nanp02aa1n02x5               g102(.a(\b[16] ), .b(\a[17] ), .o1(new_n198));
  nano22aa1d15x5               g103(.a(new_n196), .b(new_n187), .c(new_n198), .out0(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  aoai13aa1n12x5               g105(.a(new_n195), .b(new_n194), .c(new_n185), .d(new_n186), .o1(new_n201));
  aoai13aa1n04x5               g106(.a(new_n201), .b(new_n200), .c(new_n175), .d(new_n182), .o1(new_n202));
  xorb03aa1n02x5               g107(.a(new_n202), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  nand22aa1n04x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  nor042aa1n04x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  nand02aa1d08x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  norb02aa1n02x5               g113(.a(new_n208), .b(new_n207), .out0(new_n209));
  aoi112aa1n02x5               g114(.a(new_n209), .b(new_n205), .c(new_n202), .d(new_n206), .o1(new_n210));
  aoai13aa1n03x5               g115(.a(new_n209), .b(new_n205), .c(new_n202), .d(new_n206), .o1(new_n211));
  norb02aa1n02x7               g116(.a(new_n211), .b(new_n210), .out0(\s[20] ));
  nano23aa1n06x5               g117(.a(new_n205), .b(new_n207), .c(new_n208), .d(new_n206), .out0(new_n213));
  nanp02aa1n02x5               g118(.a(new_n199), .b(new_n213), .o1(new_n214));
  nona23aa1d18x5               g119(.a(new_n208), .b(new_n206), .c(new_n205), .d(new_n207), .out0(new_n215));
  aoi012aa1n12x5               g120(.a(new_n207), .b(new_n205), .c(new_n208), .o1(new_n216));
  oai012aa1d24x5               g121(.a(new_n216), .b(new_n215), .c(new_n201), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  aoai13aa1n04x5               g123(.a(new_n218), .b(new_n214), .c(new_n175), .d(new_n182), .o1(new_n219));
  xorb03aa1n02x5               g124(.a(new_n219), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n03x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  xorc02aa1n02x5               g126(.a(\a[21] ), .b(\b[20] ), .out0(new_n222));
  xorc02aa1n02x5               g127(.a(\a[22] ), .b(\b[21] ), .out0(new_n223));
  aoi112aa1n03x5               g128(.a(new_n221), .b(new_n223), .c(new_n219), .d(new_n222), .o1(new_n224));
  aoai13aa1n03x5               g129(.a(new_n223), .b(new_n221), .c(new_n219), .d(new_n222), .o1(new_n225));
  norb02aa1n02x7               g130(.a(new_n225), .b(new_n224), .out0(\s[22] ));
  inv000aa1d42x5               g131(.a(\a[21] ), .o1(new_n227));
  inv040aa1d32x5               g132(.a(\a[22] ), .o1(new_n228));
  xroi22aa1d06x4               g133(.a(new_n227), .b(\b[20] ), .c(new_n228), .d(\b[21] ), .out0(new_n229));
  nand03aa1n02x5               g134(.a(new_n229), .b(new_n199), .c(new_n213), .o1(new_n230));
  inv000aa1d42x5               g135(.a(\b[21] ), .o1(new_n231));
  oaoi03aa1n09x5               g136(.a(new_n228), .b(new_n231), .c(new_n221), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  aoi012aa1n02x5               g138(.a(new_n233), .b(new_n217), .c(new_n229), .o1(new_n234));
  aoai13aa1n04x5               g139(.a(new_n234), .b(new_n230), .c(new_n175), .d(new_n182), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  xorc02aa1n02x5               g142(.a(\a[23] ), .b(\b[22] ), .out0(new_n238));
  xorc02aa1n02x5               g143(.a(\a[24] ), .b(\b[23] ), .out0(new_n239));
  aoi112aa1n02x5               g144(.a(new_n237), .b(new_n239), .c(new_n235), .d(new_n238), .o1(new_n240));
  aoai13aa1n03x5               g145(.a(new_n239), .b(new_n237), .c(new_n235), .d(new_n238), .o1(new_n241));
  norb02aa1n02x7               g146(.a(new_n241), .b(new_n240), .out0(\s[24] ));
  nanp02aa1n06x5               g147(.a(new_n175), .b(new_n182), .o1(new_n243));
  and002aa1n02x5               g148(.a(new_n239), .b(new_n238), .o(new_n244));
  inv000aa1n02x5               g149(.a(new_n244), .o1(new_n245));
  nor002aa1n02x5               g150(.a(new_n230), .b(new_n245), .o1(new_n246));
  inv000aa1n06x5               g151(.a(new_n201), .o1(new_n247));
  inv040aa1n02x5               g152(.a(new_n216), .o1(new_n248));
  aoai13aa1n06x5               g153(.a(new_n229), .b(new_n248), .c(new_n213), .d(new_n247), .o1(new_n249));
  orn002aa1n02x5               g154(.a(\a[23] ), .b(\b[22] ), .o(new_n250));
  oao003aa1n02x5               g155(.a(\a[24] ), .b(\b[23] ), .c(new_n250), .carry(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n245), .c(new_n249), .d(new_n232), .o1(new_n252));
  xorc02aa1n12x5               g157(.a(\a[25] ), .b(\b[24] ), .out0(new_n253));
  aoai13aa1n06x5               g158(.a(new_n253), .b(new_n252), .c(new_n243), .d(new_n246), .o1(new_n254));
  aoi112aa1n02x5               g159(.a(new_n253), .b(new_n252), .c(new_n243), .d(new_n246), .o1(new_n255));
  norb02aa1n02x5               g160(.a(new_n254), .b(new_n255), .out0(\s[25] ));
  norp02aa1n02x5               g161(.a(\b[24] ), .b(\a[25] ), .o1(new_n257));
  tech160nm_fixorc02aa1n05x5   g162(.a(\a[26] ), .b(\b[25] ), .out0(new_n258));
  nona22aa1n02x5               g163(.a(new_n254), .b(new_n258), .c(new_n257), .out0(new_n259));
  inv000aa1n02x5               g164(.a(new_n257), .o1(new_n260));
  aobi12aa1n06x5               g165(.a(new_n258), .b(new_n254), .c(new_n260), .out0(new_n261));
  norb02aa1n03x4               g166(.a(new_n259), .b(new_n261), .out0(\s[26] ));
  and002aa1n06x5               g167(.a(new_n258), .b(new_n253), .o(new_n263));
  nano22aa1n03x7               g168(.a(new_n230), .b(new_n244), .c(new_n263), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n192), .c(new_n191), .d(new_n174), .o1(new_n265));
  oao003aa1n02x5               g170(.a(\a[26] ), .b(\b[25] ), .c(new_n260), .carry(new_n266));
  aobi12aa1n06x5               g171(.a(new_n266), .b(new_n252), .c(new_n263), .out0(new_n267));
  xorc02aa1n02x5               g172(.a(\a[27] ), .b(\b[26] ), .out0(new_n268));
  xnbna2aa1n03x5               g173(.a(new_n268), .b(new_n267), .c(new_n265), .out0(\s[27] ));
  norp02aa1n02x5               g174(.a(\b[26] ), .b(\a[27] ), .o1(new_n270));
  inv040aa1n03x5               g175(.a(new_n270), .o1(new_n271));
  aobi12aa1n06x5               g176(.a(new_n268), .b(new_n267), .c(new_n265), .out0(new_n272));
  xnrc02aa1n02x5               g177(.a(\b[27] ), .b(\a[28] ), .out0(new_n273));
  nano22aa1n03x5               g178(.a(new_n272), .b(new_n271), .c(new_n273), .out0(new_n274));
  aobi12aa1n06x5               g179(.a(new_n264), .b(new_n175), .c(new_n182), .out0(new_n275));
  aoai13aa1n06x5               g180(.a(new_n244), .b(new_n233), .c(new_n217), .d(new_n229), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n263), .o1(new_n277));
  aoai13aa1n06x5               g182(.a(new_n266), .b(new_n277), .c(new_n276), .d(new_n251), .o1(new_n278));
  oaih12aa1n02x5               g183(.a(new_n268), .b(new_n278), .c(new_n275), .o1(new_n279));
  tech160nm_fiaoi012aa1n05x5   g184(.a(new_n273), .b(new_n279), .c(new_n271), .o1(new_n280));
  norp02aa1n03x5               g185(.a(new_n280), .b(new_n274), .o1(\s[28] ));
  xnrc02aa1n02x5               g186(.a(\b[28] ), .b(\a[29] ), .out0(new_n282));
  norb02aa1n02x5               g187(.a(new_n268), .b(new_n273), .out0(new_n283));
  oaih12aa1n02x5               g188(.a(new_n283), .b(new_n278), .c(new_n275), .o1(new_n284));
  oao003aa1n02x5               g189(.a(\a[28] ), .b(\b[27] ), .c(new_n271), .carry(new_n285));
  tech160nm_fiaoi012aa1n02p5x5 g190(.a(new_n282), .b(new_n284), .c(new_n285), .o1(new_n286));
  aobi12aa1n02x7               g191(.a(new_n283), .b(new_n267), .c(new_n265), .out0(new_n287));
  nano22aa1n02x4               g192(.a(new_n287), .b(new_n282), .c(new_n285), .out0(new_n288));
  norp02aa1n03x5               g193(.a(new_n286), .b(new_n288), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n106), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g195(.a(new_n268), .b(new_n282), .c(new_n273), .out0(new_n291));
  oaih12aa1n02x5               g196(.a(new_n291), .b(new_n278), .c(new_n275), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .c(new_n285), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[29] ), .b(\a[30] ), .out0(new_n294));
  tech160nm_fiaoi012aa1n03p5x5 g199(.a(new_n294), .b(new_n292), .c(new_n293), .o1(new_n295));
  aobi12aa1n02x7               g200(.a(new_n291), .b(new_n267), .c(new_n265), .out0(new_n296));
  nano22aa1n03x5               g201(.a(new_n296), .b(new_n293), .c(new_n294), .out0(new_n297));
  norp02aa1n03x5               g202(.a(new_n295), .b(new_n297), .o1(\s[30] ));
  xnrc02aa1n02x5               g203(.a(\b[30] ), .b(\a[31] ), .out0(new_n299));
  norb02aa1n02x5               g204(.a(new_n291), .b(new_n294), .out0(new_n300));
  aobi12aa1n06x5               g205(.a(new_n300), .b(new_n267), .c(new_n265), .out0(new_n301));
  oao003aa1n02x5               g206(.a(\a[30] ), .b(\b[29] ), .c(new_n293), .carry(new_n302));
  nano22aa1n03x5               g207(.a(new_n301), .b(new_n299), .c(new_n302), .out0(new_n303));
  oaih12aa1n02x5               g208(.a(new_n300), .b(new_n278), .c(new_n275), .o1(new_n304));
  tech160nm_fiaoi012aa1n02p5x5 g209(.a(new_n299), .b(new_n304), .c(new_n302), .o1(new_n305));
  norp02aa1n03x5               g210(.a(new_n305), .b(new_n303), .o1(\s[31] ));
  xnbna2aa1n03x5               g211(.a(new_n107), .b(new_n101), .c(new_n102), .out0(\s[3] ));
  oaoi03aa1n02x5               g212(.a(\a[3] ), .b(\b[2] ), .c(new_n107), .o1(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g214(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g215(.a(new_n121), .b(new_n123), .c(new_n110), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[5] ), .c(new_n122), .out0(\s[6] ));
  oaib12aa1n02x5               g217(.a(new_n124), .b(new_n117), .c(new_n110), .out0(new_n313));
  nanb02aa1n02x5               g218(.a(new_n119), .b(new_n313), .out0(new_n314));
  xnrb03aa1n02x5               g219(.a(new_n314), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g220(.a(\a[7] ), .b(\b[6] ), .c(new_n314), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g222(.a(new_n191), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


