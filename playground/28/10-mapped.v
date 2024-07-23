// Benchmark "adder" written by ABC on Thu Jul 18 02:22:13 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n137, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n148, new_n149, new_n150, new_n151, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n256, new_n257, new_n258,
    new_n259, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n339, new_n340, new_n342, new_n343, new_n344, new_n345, new_n347,
    new_n349, new_n350, new_n351, new_n353, new_n354, new_n355, new_n357,
    new_n358;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv030aa1n02x5               g002(.a(new_n97), .o1(new_n98));
  nor002aa1n03x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand02aa1d16x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n04x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  norb03aa1n09x5               g006(.a(new_n100), .b(new_n99), .c(new_n101), .out0(new_n102));
  inv020aa1n04x5               g007(.a(new_n102), .o1(new_n103));
  inv040aa1d32x5               g008(.a(\a[4] ), .o1(new_n104));
  inv000aa1d42x5               g009(.a(\b[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(new_n105), .b(new_n104), .o1(new_n106));
  nand42aa1n02x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nor042aa1n12x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nanp02aa1n04x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nanb02aa1n03x5               g014(.a(new_n108), .b(new_n109), .out0(new_n110));
  nano32aa1n03x7               g015(.a(new_n110), .b(new_n106), .c(new_n107), .d(new_n100), .out0(new_n111));
  oaoi03aa1n09x5               g016(.a(new_n104), .b(new_n105), .c(new_n108), .o1(new_n112));
  inv020aa1n02x5               g017(.a(new_n112), .o1(new_n113));
  nor022aa1n04x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nand22aa1n03x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nor022aa1n04x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nand02aa1n03x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nona23aa1n03x5               g022(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n118));
  norp02aa1n06x5               g023(.a(\b[7] ), .b(\a[8] ), .o1(new_n119));
  nand22aa1n04x5               g024(.a(\b[7] ), .b(\a[8] ), .o1(new_n120));
  nor002aa1d24x5               g025(.a(\b[6] ), .b(\a[7] ), .o1(new_n121));
  nanp02aa1n03x5               g026(.a(\b[6] ), .b(\a[7] ), .o1(new_n122));
  nona23aa1n09x5               g027(.a(new_n122), .b(new_n120), .c(new_n119), .d(new_n121), .out0(new_n123));
  nor042aa1n03x5               g028(.a(new_n123), .b(new_n118), .o1(new_n124));
  aoai13aa1n06x5               g029(.a(new_n124), .b(new_n113), .c(new_n111), .d(new_n103), .o1(new_n125));
  inv030aa1n02x5               g030(.a(new_n123), .o1(new_n126));
  inv040aa1n02x5               g031(.a(new_n114), .o1(new_n127));
  aob012aa1n06x5               g032(.a(new_n127), .b(new_n116), .c(new_n115), .out0(new_n128));
  oai012aa1n02x7               g033(.a(new_n120), .b(new_n121), .c(new_n119), .o1(new_n129));
  aobi12aa1n09x5               g034(.a(new_n129), .b(new_n126), .c(new_n128), .out0(new_n130));
  nand02aa1d06x5               g035(.a(new_n125), .b(new_n130), .o1(new_n131));
  nanp02aa1n04x5               g036(.a(\b[8] ), .b(\a[9] ), .o1(new_n132));
  nanb02aa1n02x5               g037(.a(new_n97), .b(new_n132), .out0(new_n133));
  nanb02aa1n02x5               g038(.a(new_n133), .b(new_n131), .out0(new_n134));
  nor022aa1n06x5               g039(.a(\b[9] ), .b(\a[10] ), .o1(new_n135));
  nanp02aa1n04x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  nanb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(new_n137));
  xobna2aa1n03x5               g042(.a(new_n137), .b(new_n134), .c(new_n98), .out0(\s[10] ));
  nona23aa1d24x5               g043(.a(new_n136), .b(new_n132), .c(new_n97), .d(new_n135), .out0(new_n139));
  inv000aa1d42x5               g044(.a(new_n139), .o1(new_n140));
  oaoi03aa1n09x5               g045(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n141));
  nor042aa1d18x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  nand42aa1n02x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  norb02aa1n03x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  aoai13aa1n06x5               g049(.a(new_n144), .b(new_n141), .c(new_n131), .d(new_n140), .o1(new_n145));
  aoi112aa1n02x5               g050(.a(new_n144), .b(new_n141), .c(new_n131), .d(new_n140), .o1(new_n146));
  norb02aa1n02x5               g051(.a(new_n145), .b(new_n146), .out0(\s[11] ));
  inv040aa1n08x5               g052(.a(new_n142), .o1(new_n148));
  nor042aa1n02x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  norb02aa1n03x5               g055(.a(new_n150), .b(new_n149), .out0(new_n151));
  xnbna2aa1n03x5               g056(.a(new_n151), .b(new_n145), .c(new_n148), .out0(\s[12] ));
  nano22aa1n12x5               g057(.a(new_n139), .b(new_n144), .c(new_n151), .out0(new_n153));
  nano23aa1n06x5               g058(.a(new_n142), .b(new_n149), .c(new_n150), .d(new_n143), .out0(new_n154));
  oaoi03aa1n12x5               g059(.a(\a[12] ), .b(\b[11] ), .c(new_n148), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  aob012aa1n02x5               g061(.a(new_n156), .b(new_n154), .c(new_n141), .out0(new_n157));
  xnrc02aa1n12x5               g062(.a(\b[12] ), .b(\a[13] ), .out0(new_n158));
  inv000aa1d42x5               g063(.a(new_n158), .o1(new_n159));
  aoai13aa1n06x5               g064(.a(new_n159), .b(new_n157), .c(new_n131), .d(new_n153), .o1(new_n160));
  aoi112aa1n02x5               g065(.a(new_n155), .b(new_n159), .c(new_n154), .d(new_n141), .o1(new_n161));
  aobi12aa1n02x5               g066(.a(new_n161), .b(new_n131), .c(new_n153), .out0(new_n162));
  norb02aa1n02x5               g067(.a(new_n160), .b(new_n162), .out0(\s[13] ));
  orn002aa1n02x5               g068(.a(\a[13] ), .b(\b[12] ), .o(new_n164));
  tech160nm_fixnrc02aa1n04x5   g069(.a(\b[13] ), .b(\a[14] ), .out0(new_n165));
  xobna2aa1n03x5               g070(.a(new_n165), .b(new_n160), .c(new_n164), .out0(\s[14] ));
  nor042aa1n06x5               g071(.a(new_n165), .b(new_n158), .o1(new_n167));
  nano22aa1n02x4               g072(.a(new_n139), .b(new_n167), .c(new_n154), .out0(new_n168));
  aobi12aa1n02x5               g073(.a(new_n168), .b(new_n130), .c(new_n125), .out0(new_n169));
  aoai13aa1n12x5               g074(.a(new_n167), .b(new_n155), .c(new_n154), .d(new_n141), .o1(new_n170));
  oao003aa1n02x5               g075(.a(\a[14] ), .b(\b[13] ), .c(new_n164), .carry(new_n171));
  nand02aa1d06x5               g076(.a(new_n170), .b(new_n171), .o1(new_n172));
  xorc02aa1n02x5               g077(.a(\a[15] ), .b(\b[14] ), .out0(new_n173));
  aoai13aa1n06x5               g078(.a(new_n173), .b(new_n172), .c(new_n131), .d(new_n168), .o1(new_n174));
  nanb03aa1n02x5               g079(.a(new_n173), .b(new_n170), .c(new_n171), .out0(new_n175));
  oa0012aa1n02x5               g080(.a(new_n174), .b(new_n175), .c(new_n169), .o(\s[15] ));
  inv000aa1d42x5               g081(.a(\a[15] ), .o1(new_n177));
  nanb02aa1n02x5               g082(.a(\b[14] ), .b(new_n177), .out0(new_n178));
  xorc02aa1n02x5               g083(.a(\a[16] ), .b(\b[15] ), .out0(new_n179));
  xnbna2aa1n03x5               g084(.a(new_n179), .b(new_n174), .c(new_n178), .out0(\s[16] ));
  nanp02aa1n02x5               g085(.a(new_n106), .b(new_n107), .o1(new_n181));
  nanb03aa1n03x5               g086(.a(new_n108), .b(new_n109), .c(new_n100), .out0(new_n182));
  oai013aa1n02x5               g087(.a(new_n112), .b(new_n102), .c(new_n182), .d(new_n181), .o1(new_n183));
  oaib12aa1n02x5               g088(.a(new_n129), .b(new_n123), .c(new_n128), .out0(new_n184));
  aoi012aa1n02x7               g089(.a(new_n184), .b(new_n183), .c(new_n124), .o1(new_n185));
  inv020aa1n04x5               g090(.a(\a[16] ), .o1(new_n186));
  xroi22aa1d06x4               g091(.a(new_n177), .b(\b[14] ), .c(new_n186), .d(\b[15] ), .out0(new_n187));
  nand23aa1n03x5               g092(.a(new_n153), .b(new_n167), .c(new_n187), .o1(new_n188));
  nand02aa1d06x5               g093(.a(new_n172), .b(new_n187), .o1(new_n189));
  oao003aa1n02x5               g094(.a(\a[16] ), .b(\b[15] ), .c(new_n178), .carry(new_n190));
  oai112aa1n06x5               g095(.a(new_n189), .b(new_n190), .c(new_n185), .d(new_n188), .o1(new_n191));
  tech160nm_fixnrc02aa1n04x5   g096(.a(\b[16] ), .b(\a[17] ), .out0(new_n192));
  aoi012aa1n12x5               g097(.a(new_n188), .b(new_n125), .c(new_n130), .o1(new_n193));
  nona23aa1n02x4               g098(.a(new_n189), .b(new_n190), .c(new_n193), .d(new_n192), .out0(new_n194));
  aob012aa1n02x5               g099(.a(new_n194), .b(new_n191), .c(new_n192), .out0(\s[17] ));
  nor042aa1n09x5               g100(.a(\b[16] ), .b(\a[17] ), .o1(new_n196));
  nona23aa1n03x5               g101(.a(new_n189), .b(new_n190), .c(new_n193), .d(new_n196), .out0(new_n197));
  inv000aa1d42x5               g102(.a(\b[16] ), .o1(new_n198));
  obai22aa1n02x7               g103(.a(\a[17] ), .b(new_n198), .c(\b[17] ), .d(\a[18] ), .out0(new_n199));
  aoi012aa1n02x5               g104(.a(new_n199), .b(\a[18] ), .c(\b[17] ), .o1(new_n200));
  xnrc02aa1n02x5               g105(.a(\b[17] ), .b(\a[18] ), .out0(new_n201));
  oaib12aa1n03x5               g106(.a(new_n197), .b(new_n198), .c(\a[17] ), .out0(new_n202));
  aoi022aa1n02x7               g107(.a(new_n202), .b(new_n201), .c(new_n197), .d(new_n200), .o1(\s[18] ));
  inv000aa1n02x5               g108(.a(new_n187), .o1(new_n204));
  aoai13aa1n09x5               g109(.a(new_n190), .b(new_n204), .c(new_n170), .d(new_n171), .o1(new_n205));
  nor042aa1n03x5               g110(.a(new_n201), .b(new_n192), .o1(new_n206));
  tech160nm_fioai012aa1n05x5   g111(.a(new_n206), .b(new_n205), .c(new_n193), .o1(new_n207));
  inv000aa1d42x5               g112(.a(\a[18] ), .o1(new_n208));
  inv000aa1d42x5               g113(.a(\b[17] ), .o1(new_n209));
  tech160nm_fioaoi03aa1n03p5x5 g114(.a(new_n208), .b(new_n209), .c(new_n196), .o1(new_n210));
  xorc02aa1n12x5               g115(.a(\a[19] ), .b(\b[18] ), .out0(new_n211));
  xnbna2aa1n03x5               g116(.a(new_n211), .b(new_n207), .c(new_n210), .out0(\s[19] ));
  xnrc02aa1n02x5               g117(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1n02x5               g118(.a(new_n210), .o1(new_n214));
  aoai13aa1n03x5               g119(.a(new_n211), .b(new_n214), .c(new_n191), .d(new_n206), .o1(new_n215));
  xorc02aa1n02x5               g120(.a(\a[20] ), .b(\b[19] ), .out0(new_n216));
  inv000aa1d42x5               g121(.a(\a[19] ), .o1(new_n217));
  nanb02aa1n12x5               g122(.a(\b[18] ), .b(new_n217), .out0(new_n218));
  norb02aa1n02x5               g123(.a(new_n218), .b(new_n216), .out0(new_n219));
  inv000aa1d42x5               g124(.a(new_n211), .o1(new_n220));
  aoai13aa1n02x7               g125(.a(new_n218), .b(new_n220), .c(new_n207), .d(new_n210), .o1(new_n221));
  aoi022aa1n03x5               g126(.a(new_n221), .b(new_n216), .c(new_n215), .d(new_n219), .o1(\s[20] ));
  and003aa1n02x5               g127(.a(new_n206), .b(new_n216), .c(new_n211), .o(new_n223));
  tech160nm_fioai012aa1n05x5   g128(.a(new_n223), .b(new_n205), .c(new_n193), .o1(new_n224));
  inv020aa1n04x5               g129(.a(\a[20] ), .o1(new_n225));
  xroi22aa1d06x4               g130(.a(new_n217), .b(\b[18] ), .c(new_n225), .d(\b[19] ), .out0(new_n226));
  oao003aa1n06x5               g131(.a(\a[20] ), .b(\b[19] ), .c(new_n218), .carry(new_n227));
  aobi12aa1d24x5               g132(.a(new_n227), .b(new_n226), .c(new_n214), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  xorc02aa1n12x5               g134(.a(\a[21] ), .b(\b[20] ), .out0(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n229), .c(new_n191), .d(new_n223), .o1(new_n231));
  nano22aa1n02x4               g136(.a(new_n210), .b(new_n211), .c(new_n216), .out0(new_n232));
  inv000aa1d42x5               g137(.a(new_n230), .o1(new_n233));
  nano22aa1n02x4               g138(.a(new_n232), .b(new_n227), .c(new_n233), .out0(new_n234));
  aobi12aa1n02x7               g139(.a(new_n231), .b(new_n234), .c(new_n224), .out0(\s[21] ));
  xorc02aa1n12x5               g140(.a(\a[22] ), .b(\b[21] ), .out0(new_n236));
  nor042aa1n04x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  norp02aa1n02x5               g142(.a(new_n236), .b(new_n237), .o1(new_n238));
  inv000aa1n02x5               g143(.a(new_n237), .o1(new_n239));
  aoai13aa1n03x5               g144(.a(new_n239), .b(new_n233), .c(new_n224), .d(new_n228), .o1(new_n240));
  aoi022aa1n03x5               g145(.a(new_n240), .b(new_n236), .c(new_n231), .d(new_n238), .o1(\s[22] ));
  nano32aa1n02x4               g146(.a(new_n233), .b(new_n226), .c(new_n206), .d(new_n236), .out0(new_n242));
  tech160nm_fioai012aa1n03p5x5 g147(.a(new_n242), .b(new_n205), .c(new_n193), .o1(new_n243));
  and002aa1n02x5               g148(.a(new_n236), .b(new_n230), .o(new_n244));
  norb02aa1n06x4               g149(.a(new_n244), .b(new_n228), .out0(new_n245));
  oaoi03aa1n02x5               g150(.a(\a[22] ), .b(\b[21] ), .c(new_n239), .o1(new_n246));
  norp02aa1n02x5               g151(.a(new_n245), .b(new_n246), .o1(new_n247));
  nor042aa1n03x5               g152(.a(\b[22] ), .b(\a[23] ), .o1(new_n248));
  nand02aa1d16x5               g153(.a(\b[22] ), .b(\a[23] ), .o1(new_n249));
  norb02aa1n06x5               g154(.a(new_n249), .b(new_n248), .out0(new_n250));
  aoi112aa1n02x5               g155(.a(\b[20] ), .b(\a[21] ), .c(\a[22] ), .d(\b[21] ), .o1(new_n251));
  oab012aa1n02x4               g156(.a(new_n248), .b(\a[22] ), .c(\b[21] ), .out0(new_n252));
  nona23aa1n09x5               g157(.a(new_n252), .b(new_n249), .c(new_n245), .d(new_n251), .out0(new_n253));
  nanb02aa1n03x5               g158(.a(new_n253), .b(new_n243), .out0(new_n254));
  aoai13aa1n02x5               g159(.a(new_n254), .b(new_n250), .c(new_n243), .d(new_n247), .o1(\s[23] ));
  oai012aa1n02x5               g160(.a(new_n249), .b(\b[23] ), .c(\a[24] ), .o1(new_n256));
  aoi012aa1n02x5               g161(.a(new_n256), .b(\a[24] ), .c(\b[23] ), .o1(new_n257));
  xnrc02aa1n03x5               g162(.a(\b[23] ), .b(\a[24] ), .out0(new_n258));
  aoai13aa1n03x5               g163(.a(new_n249), .b(new_n253), .c(new_n191), .d(new_n242), .o1(new_n259));
  aoi022aa1n03x5               g164(.a(new_n259), .b(new_n258), .c(new_n254), .d(new_n257), .o1(\s[24] ));
  norb02aa1n06x5               g165(.a(new_n250), .b(new_n258), .out0(new_n261));
  nand23aa1n04x5               g166(.a(new_n261), .b(new_n230), .c(new_n236), .o1(new_n262));
  nano22aa1n02x4               g167(.a(new_n262), .b(new_n206), .c(new_n226), .out0(new_n263));
  tech160nm_fioai012aa1n04x5   g168(.a(new_n263), .b(new_n205), .c(new_n193), .o1(new_n264));
  inv000aa1n06x5               g169(.a(new_n262), .o1(new_n265));
  oaib12aa1n06x5               g170(.a(new_n265), .b(new_n232), .c(new_n227), .out0(new_n266));
  orn002aa1n02x5               g171(.a(\a[23] ), .b(\b[22] ), .o(new_n267));
  oaoi03aa1n02x5               g172(.a(\a[24] ), .b(\b[23] ), .c(new_n267), .o1(new_n268));
  aoi012aa1n12x5               g173(.a(new_n268), .b(new_n261), .c(new_n246), .o1(new_n269));
  tech160nm_fioai012aa1n03p5x5 g174(.a(new_n269), .b(new_n228), .c(new_n262), .o1(new_n270));
  inv000aa1n02x5               g175(.a(new_n270), .o1(new_n271));
  xorc02aa1n12x5               g176(.a(\a[25] ), .b(\b[24] ), .out0(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  aoi012aa1n03x5               g178(.a(new_n273), .b(new_n264), .c(new_n271), .o1(new_n274));
  aoi112aa1n02x5               g179(.a(new_n268), .b(new_n272), .c(new_n261), .d(new_n246), .o1(new_n275));
  aoi013aa1n02x4               g180(.a(new_n274), .b(new_n266), .c(new_n264), .d(new_n275), .o1(\s[25] ));
  aoai13aa1n03x5               g181(.a(new_n272), .b(new_n270), .c(new_n191), .d(new_n263), .o1(new_n277));
  tech160nm_fixorc02aa1n03p5x5 g182(.a(\a[26] ), .b(\b[25] ), .out0(new_n278));
  nor042aa1n04x5               g183(.a(\b[24] ), .b(\a[25] ), .o1(new_n279));
  norp02aa1n02x5               g184(.a(new_n278), .b(new_n279), .o1(new_n280));
  inv000aa1n03x5               g185(.a(new_n279), .o1(new_n281));
  aoai13aa1n03x5               g186(.a(new_n281), .b(new_n273), .c(new_n264), .d(new_n271), .o1(new_n282));
  aoi022aa1n03x5               g187(.a(new_n282), .b(new_n278), .c(new_n277), .d(new_n280), .o1(\s[26] ));
  and002aa1n12x5               g188(.a(new_n278), .b(new_n272), .o(new_n284));
  nano32aa1n03x7               g189(.a(new_n262), .b(new_n284), .c(new_n206), .d(new_n226), .out0(new_n285));
  tech160nm_fioai012aa1n05x5   g190(.a(new_n285), .b(new_n205), .c(new_n193), .o1(new_n286));
  oaoi03aa1n09x5               g191(.a(\a[26] ), .b(\b[25] ), .c(new_n281), .o1(new_n287));
  tech160nm_fiaoi012aa1n05x5   g192(.a(new_n287), .b(new_n270), .c(new_n284), .o1(new_n288));
  nor002aa1n02x5               g193(.a(\b[26] ), .b(\a[27] ), .o1(new_n289));
  nand42aa1n08x5               g194(.a(\b[26] ), .b(\a[27] ), .o1(new_n290));
  norb02aa1n06x4               g195(.a(new_n290), .b(new_n289), .out0(new_n291));
  inv000aa1d42x5               g196(.a(new_n284), .o1(new_n292));
  norb02aa1n02x5               g197(.a(new_n291), .b(new_n287), .out0(new_n293));
  oai112aa1n03x5               g198(.a(new_n286), .b(new_n293), .c(new_n292), .d(new_n271), .o1(new_n294));
  aoai13aa1n02x5               g199(.a(new_n294), .b(new_n291), .c(new_n286), .d(new_n288), .o1(\s[27] ));
  norp02aa1n02x5               g200(.a(new_n287), .b(new_n289), .o1(new_n296));
  oai112aa1n03x5               g201(.a(new_n286), .b(new_n296), .c(new_n292), .d(new_n271), .o1(new_n297));
  oai012aa1n02x5               g202(.a(new_n290), .b(\b[27] ), .c(\a[28] ), .o1(new_n298));
  aoi012aa1n02x5               g203(.a(new_n298), .b(\a[28] ), .c(\b[27] ), .o1(new_n299));
  xnrc02aa1n12x5               g204(.a(\b[27] ), .b(\a[28] ), .out0(new_n300));
  aoai13aa1n02x5               g205(.a(new_n296), .b(new_n292), .c(new_n266), .d(new_n269), .o1(new_n301));
  aoai13aa1n03x5               g206(.a(new_n290), .b(new_n301), .c(new_n191), .d(new_n285), .o1(new_n302));
  aoi022aa1n03x5               g207(.a(new_n302), .b(new_n300), .c(new_n297), .d(new_n299), .o1(\s[28] ));
  inv000aa1d42x5               g208(.a(new_n287), .o1(new_n304));
  aoai13aa1n04x5               g209(.a(new_n304), .b(new_n292), .c(new_n266), .d(new_n269), .o1(new_n305));
  norb02aa1n03x5               g210(.a(new_n291), .b(new_n300), .out0(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n305), .c(new_n191), .d(new_n285), .o1(new_n307));
  inv000aa1n02x5               g212(.a(new_n306), .o1(new_n308));
  orn002aa1n03x5               g213(.a(\a[27] ), .b(\b[26] ), .o(new_n309));
  oao003aa1n03x5               g214(.a(\a[28] ), .b(\b[27] ), .c(new_n309), .carry(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n308), .c(new_n286), .d(new_n288), .o1(new_n311));
  xorc02aa1n02x5               g216(.a(\a[29] ), .b(\b[28] ), .out0(new_n312));
  norb02aa1n02x5               g217(.a(new_n310), .b(new_n312), .out0(new_n313));
  aoi022aa1n03x5               g218(.a(new_n311), .b(new_n312), .c(new_n307), .d(new_n313), .o1(\s[29] ));
  xorb03aa1n02x5               g219(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g220(.a(new_n300), .b(new_n312), .c(new_n291), .out0(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n305), .c(new_n191), .d(new_n285), .o1(new_n317));
  inv000aa1d42x5               g222(.a(new_n316), .o1(new_n318));
  tech160nm_fioaoi03aa1n03p5x5 g223(.a(\a[29] ), .b(\b[28] ), .c(new_n310), .o1(new_n319));
  inv000aa1d42x5               g224(.a(new_n319), .o1(new_n320));
  aoai13aa1n03x5               g225(.a(new_n320), .b(new_n318), .c(new_n286), .d(new_n288), .o1(new_n321));
  xorc02aa1n02x5               g226(.a(\a[30] ), .b(\b[29] ), .out0(new_n322));
  aoi012aa1n02x5               g227(.a(new_n310), .b(\a[29] ), .c(\b[28] ), .o1(new_n323));
  oabi12aa1n02x5               g228(.a(new_n322), .b(\a[29] ), .c(\b[28] ), .out0(new_n324));
  norp02aa1n02x5               g229(.a(new_n323), .b(new_n324), .o1(new_n325));
  aoi022aa1n03x5               g230(.a(new_n321), .b(new_n322), .c(new_n317), .d(new_n325), .o1(\s[30] ));
  nano22aa1n02x5               g231(.a(new_n308), .b(new_n312), .c(new_n322), .out0(new_n327));
  aoai13aa1n03x5               g232(.a(new_n327), .b(new_n305), .c(new_n191), .d(new_n285), .o1(new_n328));
  xorc02aa1n02x5               g233(.a(\a[31] ), .b(\b[30] ), .out0(new_n329));
  inv000aa1d42x5               g234(.a(\a[30] ), .o1(new_n330));
  inv000aa1d42x5               g235(.a(\b[29] ), .o1(new_n331));
  oabi12aa1n02x5               g236(.a(new_n329), .b(\a[30] ), .c(\b[29] ), .out0(new_n332));
  oaoi13aa1n04x5               g237(.a(new_n332), .b(new_n319), .c(new_n330), .d(new_n331), .o1(new_n333));
  inv000aa1n02x5               g238(.a(new_n327), .o1(new_n334));
  oaoi03aa1n03x5               g239(.a(new_n330), .b(new_n331), .c(new_n319), .o1(new_n335));
  aoai13aa1n03x5               g240(.a(new_n335), .b(new_n334), .c(new_n286), .d(new_n288), .o1(new_n336));
  aoi022aa1n03x5               g241(.a(new_n336), .b(new_n329), .c(new_n328), .d(new_n333), .o1(\s[31] ));
  xnbna2aa1n03x5               g242(.a(new_n110), .b(new_n103), .c(new_n100), .out0(\s[3] ));
  oai012aa1n02x5               g243(.a(new_n100), .b(new_n99), .c(new_n101), .o1(new_n339));
  nanb03aa1n02x5               g244(.a(new_n108), .b(new_n339), .c(new_n109), .out0(new_n340));
  xnbna2aa1n03x5               g245(.a(new_n181), .b(new_n340), .c(new_n109), .out0(\s[4] ));
  oaib12aa1n02x5               g246(.a(new_n183), .b(new_n116), .c(new_n117), .out0(new_n342));
  oai022aa1n02x5               g247(.a(\a[4] ), .b(\b[3] ), .c(\b[4] ), .d(\a[5] ), .o1(new_n343));
  aoi122aa1n02x5               g248(.a(new_n343), .b(\a[5] ), .c(\b[4] ), .d(new_n108), .e(new_n107), .o1(new_n344));
  oai013aa1n02x4               g249(.a(new_n344), .b(new_n102), .c(new_n182), .d(new_n181), .o1(new_n345));
  nanp02aa1n02x5               g250(.a(new_n342), .b(new_n345), .o1(\s[5] ));
  norb02aa1n02x5               g251(.a(new_n115), .b(new_n114), .out0(new_n347));
  xobna2aa1n03x5               g252(.a(new_n347), .b(new_n345), .c(new_n117), .out0(\s[6] ));
  nanb02aa1n02x5               g253(.a(new_n121), .b(new_n122), .out0(new_n349));
  inv000aa1d42x5               g254(.a(new_n349), .o1(new_n350));
  nanp03aa1n02x5               g255(.a(new_n345), .b(new_n347), .c(new_n117), .o1(new_n351));
  xnbna2aa1n03x5               g256(.a(new_n350), .b(new_n351), .c(new_n127), .out0(\s[7] ));
  nanb02aa1n02x5               g257(.a(new_n119), .b(new_n120), .out0(new_n353));
  inv000aa1d42x5               g258(.a(new_n121), .o1(new_n354));
  aob012aa1n02x5               g259(.a(new_n350), .b(new_n351), .c(new_n127), .out0(new_n355));
  xobna2aa1n03x5               g260(.a(new_n353), .b(new_n355), .c(new_n354), .out0(\s[8] ));
  nanp02aa1n02x5               g261(.a(new_n129), .b(new_n133), .o1(new_n357));
  aoi012aa1n02x5               g262(.a(new_n357), .b(new_n126), .c(new_n128), .o1(new_n358));
  aboi22aa1n03x5               g263(.a(new_n133), .b(new_n131), .c(new_n358), .d(new_n125), .out0(\s[9] ));
endmodule


