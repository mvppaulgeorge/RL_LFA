// Benchmark "adder" written by ABC on Wed Jul 17 19:42:23 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n173, new_n174, new_n175, new_n176, new_n177,
    new_n178, new_n179, new_n180, new_n181, new_n183, new_n184, new_n185,
    new_n186, new_n187, new_n188, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n195, new_n196, new_n197, new_n198, new_n199, new_n200,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n211, new_n212, new_n213, new_n214, new_n215,
    new_n217, new_n218, new_n219, new_n220, new_n221, new_n222, new_n223,
    new_n224, new_n226, new_n227, new_n228, new_n229, new_n230, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n238, new_n239, new_n240,
    new_n241, new_n242, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n252, new_n253, new_n254, new_n255,
    new_n256, new_n258, new_n259, new_n260, new_n261, new_n262, new_n263,
    new_n264, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n272, new_n273, new_n274, new_n275, new_n276, new_n277, new_n279,
    new_n280, new_n281, new_n282, new_n283, new_n284, new_n285, new_n286,
    new_n287, new_n289, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n304, new_n305, new_n306, new_n307, new_n308, new_n309,
    new_n311, new_n312, new_n313, new_n314, new_n315, new_n316, new_n317,
    new_n318, new_n319, new_n320, new_n322, new_n323, new_n324, new_n325,
    new_n326, new_n327, new_n328, new_n329, new_n330, new_n331, new_n333,
    new_n334, new_n335, new_n336, new_n337, new_n338, new_n339, new_n340,
    new_n342, new_n344, new_n345, new_n346, new_n347, new_n348, new_n349,
    new_n350, new_n351, new_n352, new_n353, new_n355, new_n356, new_n357,
    new_n358, new_n359, new_n360, new_n361, new_n362, new_n365, new_n367,
    new_n368, new_n369, new_n370, new_n371, new_n373, new_n374, new_n375,
    new_n377, new_n378, new_n380, new_n381, new_n383, new_n384;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  oai112aa1n04x5               g002(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n98));
  nand42aa1n03x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  norp02aa1n02x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nanb03aa1n02x5               g006(.a(new_n100), .b(new_n101), .c(new_n99), .out0(new_n102));
  nanp02aa1n04x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  orn002aa1n24x5               g008(.a(\a[4] ), .b(\b[3] ), .o(new_n104));
  oai112aa1n06x5               g009(.a(new_n104), .b(new_n103), .c(\b[2] ), .d(\a[3] ), .o1(new_n105));
  inv000aa1d42x5               g010(.a(new_n105), .o1(new_n106));
  oaib12aa1n09x5               g011(.a(new_n106), .b(new_n102), .c(new_n98), .out0(new_n107));
  inv000aa1d42x5               g012(.a(\a[7] ), .o1(new_n108));
  aoi022aa1d24x5               g013(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n109));
  oaib12aa1n06x5               g014(.a(new_n109), .b(\b[6] ), .c(new_n108), .out0(new_n110));
  aoi022aa1n06x5               g015(.a(\b[4] ), .b(\a[5] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n111));
  oai022aa1n09x5               g016(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n112));
  inv000aa1d42x5               g017(.a(\a[8] ), .o1(new_n113));
  inv000aa1d42x5               g018(.a(\b[7] ), .o1(new_n114));
  aoi022aa1n02x5               g019(.a(new_n114), .b(new_n113), .c(\a[6] ), .d(\b[5] ), .o1(new_n115));
  nano23aa1n06x5               g020(.a(new_n110), .b(new_n112), .c(new_n115), .d(new_n111), .out0(new_n116));
  nand42aa1d28x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  oai012aa1d24x5               g022(.a(new_n117), .b(\b[7] ), .c(\a[8] ), .o1(new_n118));
  inv000aa1d42x5               g023(.a(new_n117), .o1(new_n119));
  norp02aa1n02x5               g024(.a(new_n112), .b(new_n119), .o1(new_n120));
  nor042aa1n03x5               g025(.a(\b[6] ), .b(\a[7] ), .o1(new_n121));
  oaoi03aa1n02x5               g026(.a(new_n113), .b(new_n114), .c(new_n121), .o1(new_n122));
  oai013aa1n06x5               g027(.a(new_n122), .b(new_n120), .c(new_n110), .d(new_n118), .o1(new_n123));
  xorc02aa1n02x5               g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  aoai13aa1n06x5               g029(.a(new_n124), .b(new_n123), .c(new_n116), .d(new_n107), .o1(new_n125));
  xorc02aa1n02x5               g030(.a(\a[10] ), .b(\b[9] ), .out0(new_n126));
  and002aa1n24x5               g031(.a(\b[9] ), .b(\a[10] ), .o(new_n127));
  oai022aa1d24x5               g032(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n128));
  nona22aa1n02x4               g033(.a(new_n125), .b(new_n127), .c(new_n128), .out0(new_n129));
  aoai13aa1n02x5               g034(.a(new_n129), .b(new_n126), .c(new_n97), .d(new_n125), .o1(\s[10] ));
  inv000aa1d42x5               g035(.a(new_n128), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(new_n125), .b(new_n131), .o1(new_n132));
  nand42aa1n03x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nor002aa1n12x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  aoi012aa1n12x5               g039(.a(new_n134), .b(\a[10] ), .c(\b[9] ), .o1(new_n135));
  nanp02aa1n02x5               g040(.a(new_n135), .b(new_n133), .o1(new_n136));
  nanb02aa1n02x5               g041(.a(new_n136), .b(new_n132), .out0(new_n137));
  inv000aa1d42x5               g042(.a(new_n127), .o1(new_n138));
  inv000aa1d42x5               g043(.a(new_n134), .o1(new_n139));
  aoi022aa1n02x5               g044(.a(new_n132), .b(new_n138), .c(new_n139), .d(new_n133), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n137), .b(new_n140), .out0(\s[11] ));
  nor042aa1n09x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand42aa1n08x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  inv000aa1d42x5               g049(.a(new_n142), .o1(new_n145));
  aoi012aa1n02x5               g050(.a(new_n134), .b(new_n145), .c(new_n143), .o1(new_n146));
  aoai13aa1n02x5               g051(.a(new_n139), .b(new_n136), .c(new_n125), .d(new_n131), .o1(new_n147));
  aoi022aa1n02x5               g052(.a(new_n137), .b(new_n146), .c(new_n147), .d(new_n144), .o1(\s[12] ));
  nano23aa1n03x5               g053(.a(new_n142), .b(new_n134), .c(new_n143), .d(new_n133), .out0(new_n149));
  nanp02aa1n02x5               g054(.a(new_n126), .b(new_n124), .o1(new_n150));
  norb02aa1n02x5               g055(.a(new_n149), .b(new_n150), .out0(new_n151));
  aoai13aa1n02x5               g056(.a(new_n151), .b(new_n123), .c(new_n116), .d(new_n107), .o1(new_n152));
  nano22aa1n03x7               g057(.a(new_n142), .b(new_n133), .c(new_n143), .out0(new_n153));
  oai112aa1n06x5               g058(.a(new_n153), .b(new_n135), .c(new_n128), .d(new_n127), .o1(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  aoi012aa1d18x5               g060(.a(new_n142), .b(new_n134), .c(new_n143), .o1(new_n156));
  inv000aa1d42x5               g061(.a(new_n156), .o1(new_n157));
  nona22aa1n02x5               g062(.a(new_n152), .b(new_n155), .c(new_n157), .out0(new_n158));
  nor042aa1n06x5               g063(.a(\b[12] ), .b(\a[13] ), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  norb02aa1n02x5               g065(.a(new_n160), .b(new_n159), .out0(new_n161));
  nano22aa1n02x4               g066(.a(new_n161), .b(new_n154), .c(new_n156), .out0(new_n162));
  aoi022aa1n02x5               g067(.a(new_n158), .b(new_n161), .c(new_n152), .d(new_n162), .o1(\s[13] ));
  inv000aa1d42x5               g068(.a(new_n159), .o1(new_n164));
  norb02aa1n03x5               g069(.a(new_n101), .b(new_n100), .out0(new_n165));
  aoi013aa1n06x4               g070(.a(new_n105), .b(new_n165), .c(new_n99), .d(new_n98), .o1(new_n166));
  nanp02aa1n12x5               g071(.a(\b[6] ), .b(\a[7] ), .o1(new_n167));
  aob012aa1n02x5               g072(.a(new_n167), .b(\b[7] ), .c(\a[8] ), .out0(new_n168));
  norp02aa1n02x5               g073(.a(new_n168), .b(new_n121), .o1(new_n169));
  norp02aa1n02x5               g074(.a(new_n118), .b(new_n112), .o1(new_n170));
  nanp03aa1n02x5               g075(.a(new_n169), .b(new_n170), .c(new_n111), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n112), .b(new_n117), .out0(new_n172));
  norp03aa1n02x5               g077(.a(new_n168), .b(new_n118), .c(new_n121), .o1(new_n173));
  inv000aa1n02x5               g078(.a(new_n122), .o1(new_n174));
  aoi012aa1n02x5               g079(.a(new_n174), .b(new_n173), .c(new_n172), .o1(new_n175));
  oai012aa1n12x5               g080(.a(new_n175), .b(new_n166), .c(new_n171), .o1(new_n176));
  nanp02aa1n09x5               g081(.a(new_n154), .b(new_n156), .o1(new_n177));
  aoai13aa1n02x5               g082(.a(new_n161), .b(new_n177), .c(new_n176), .d(new_n151), .o1(new_n178));
  nor002aa1n03x5               g083(.a(\b[13] ), .b(\a[14] ), .o1(new_n179));
  nanp02aa1n04x5               g084(.a(\b[13] ), .b(\a[14] ), .o1(new_n180));
  norb02aa1n06x4               g085(.a(new_n180), .b(new_n179), .out0(new_n181));
  xnbna2aa1n03x5               g086(.a(new_n181), .b(new_n178), .c(new_n164), .out0(\s[14] ));
  nano23aa1n09x5               g087(.a(new_n159), .b(new_n179), .c(new_n180), .d(new_n160), .out0(new_n183));
  aoai13aa1n03x5               g088(.a(new_n183), .b(new_n177), .c(new_n176), .d(new_n151), .o1(new_n184));
  aoi012aa1d18x5               g089(.a(new_n179), .b(new_n159), .c(new_n180), .o1(new_n185));
  nor042aa1n04x5               g090(.a(\b[14] ), .b(\a[15] ), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(\b[14] ), .b(\a[15] ), .o1(new_n187));
  norb02aa1n03x5               g092(.a(new_n187), .b(new_n186), .out0(new_n188));
  xnbna2aa1n03x5               g093(.a(new_n188), .b(new_n184), .c(new_n185), .out0(\s[15] ));
  inv000aa1d42x5               g094(.a(new_n185), .o1(new_n190));
  aoai13aa1n02x5               g095(.a(new_n188), .b(new_n190), .c(new_n158), .d(new_n183), .o1(new_n191));
  nor042aa1n02x5               g096(.a(\b[15] ), .b(\a[16] ), .o1(new_n192));
  nand42aa1n03x5               g097(.a(\b[15] ), .b(\a[16] ), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n193), .b(new_n192), .out0(new_n194));
  inv000aa1d42x5               g099(.a(\a[15] ), .o1(new_n195));
  inv000aa1d42x5               g100(.a(\b[14] ), .o1(new_n196));
  aboi22aa1n03x5               g101(.a(new_n192), .b(new_n193), .c(new_n195), .d(new_n196), .out0(new_n197));
  inv000aa1d42x5               g102(.a(new_n186), .o1(new_n198));
  inv000aa1n02x5               g103(.a(new_n188), .o1(new_n199));
  aoai13aa1n02x5               g104(.a(new_n198), .b(new_n199), .c(new_n184), .d(new_n185), .o1(new_n200));
  aoi022aa1n02x5               g105(.a(new_n200), .b(new_n194), .c(new_n191), .d(new_n197), .o1(\s[16] ));
  nano23aa1n09x5               g106(.a(new_n186), .b(new_n192), .c(new_n193), .d(new_n187), .out0(new_n202));
  nand02aa1n02x5               g107(.a(new_n202), .b(new_n183), .o1(new_n203));
  nano32aa1n03x7               g108(.a(new_n203), .b(new_n149), .c(new_n126), .d(new_n124), .out0(new_n204));
  aoai13aa1n02x5               g109(.a(new_n204), .b(new_n123), .c(new_n107), .d(new_n116), .o1(new_n205));
  aoi012aa1n06x5               g110(.a(new_n123), .b(new_n116), .c(new_n107), .o1(new_n206));
  nano32aa1n03x7               g111(.a(new_n199), .b(new_n194), .c(new_n161), .d(new_n181), .out0(new_n207));
  nanp02aa1n02x5               g112(.a(new_n151), .b(new_n207), .o1(new_n208));
  aoai13aa1n02x5               g113(.a(new_n193), .b(new_n192), .c(new_n195), .d(new_n196), .o1(new_n209));
  aobi12aa1n06x5               g114(.a(new_n209), .b(new_n202), .c(new_n190), .out0(new_n210));
  aobi12aa1n06x5               g115(.a(new_n210), .b(new_n177), .c(new_n207), .out0(new_n211));
  oaih12aa1n06x5               g116(.a(new_n211), .b(new_n206), .c(new_n208), .o1(new_n212));
  xorc02aa1n02x5               g117(.a(\a[17] ), .b(\b[16] ), .out0(new_n213));
  nanb02aa1n02x5               g118(.a(new_n213), .b(new_n209), .out0(new_n214));
  aoi122aa1n02x5               g119(.a(new_n214), .b(new_n190), .c(new_n202), .d(new_n177), .e(new_n207), .o1(new_n215));
  aoi022aa1n02x5               g120(.a(new_n212), .b(new_n213), .c(new_n215), .d(new_n205), .o1(\s[17] ));
  inv000aa1d42x5               g121(.a(\a[17] ), .o1(new_n217));
  inv000aa1d42x5               g122(.a(\b[16] ), .o1(new_n218));
  nanp02aa1n02x5               g123(.a(new_n218), .b(new_n217), .o1(new_n219));
  aoai13aa1n12x5               g124(.a(new_n210), .b(new_n203), .c(new_n154), .d(new_n156), .o1(new_n220));
  aoai13aa1n03x5               g125(.a(new_n213), .b(new_n220), .c(new_n176), .d(new_n204), .o1(new_n221));
  nor002aa1n02x5               g126(.a(\b[17] ), .b(\a[18] ), .o1(new_n222));
  nanp02aa1n12x5               g127(.a(\b[17] ), .b(\a[18] ), .o1(new_n223));
  norb02aa1n03x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  xnbna2aa1n03x5               g129(.a(new_n224), .b(new_n221), .c(new_n219), .out0(\s[18] ));
  xnrc02aa1n02x5               g130(.a(\b[16] ), .b(\a[17] ), .out0(new_n226));
  norb02aa1n02x5               g131(.a(new_n224), .b(new_n226), .out0(new_n227));
  aoai13aa1n06x5               g132(.a(new_n227), .b(new_n220), .c(new_n176), .d(new_n204), .o1(new_n228));
  aoi013aa1n06x4               g133(.a(new_n222), .b(new_n223), .c(new_n217), .d(new_n218), .o1(new_n229));
  tech160nm_fixorc02aa1n03p5x5 g134(.a(\a[19] ), .b(\b[18] ), .out0(new_n230));
  xnbna2aa1n03x5               g135(.a(new_n230), .b(new_n228), .c(new_n229), .out0(\s[19] ));
  xnrc02aa1n02x5               g136(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  oaoi03aa1n02x5               g137(.a(\a[18] ), .b(\b[17] ), .c(new_n219), .o1(new_n233));
  aoai13aa1n02x5               g138(.a(new_n230), .b(new_n233), .c(new_n212), .d(new_n227), .o1(new_n234));
  nor042aa1n06x5               g139(.a(\b[19] ), .b(\a[20] ), .o1(new_n235));
  nand02aa1d12x5               g140(.a(\b[19] ), .b(\a[20] ), .o1(new_n236));
  norb02aa1n02x5               g141(.a(new_n236), .b(new_n235), .out0(new_n237));
  nor042aa1n06x5               g142(.a(\b[18] ), .b(\a[19] ), .o1(new_n238));
  aoib12aa1n02x5               g143(.a(new_n238), .b(new_n236), .c(new_n235), .out0(new_n239));
  inv000aa1d42x5               g144(.a(new_n238), .o1(new_n240));
  xnrc02aa1n12x5               g145(.a(\b[18] ), .b(\a[19] ), .out0(new_n241));
  aoai13aa1n02x7               g146(.a(new_n240), .b(new_n241), .c(new_n228), .d(new_n229), .o1(new_n242));
  aoi022aa1n02x7               g147(.a(new_n242), .b(new_n237), .c(new_n234), .d(new_n239), .o1(\s[20] ));
  nano23aa1n02x4               g148(.a(new_n226), .b(new_n241), .c(new_n237), .d(new_n224), .out0(new_n244));
  aoai13aa1n06x5               g149(.a(new_n244), .b(new_n220), .c(new_n176), .d(new_n204), .o1(new_n245));
  nanb02aa1n12x5               g150(.a(new_n235), .b(new_n236), .out0(new_n246));
  aoi012aa1d18x5               g151(.a(new_n235), .b(new_n238), .c(new_n236), .o1(new_n247));
  oai013aa1d12x5               g152(.a(new_n247), .b(new_n229), .c(new_n241), .d(new_n246), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n248), .o1(new_n249));
  nanp02aa1n02x5               g154(.a(new_n245), .b(new_n249), .o1(new_n250));
  nor042aa1n12x5               g155(.a(\b[20] ), .b(\a[21] ), .o1(new_n251));
  nanp02aa1n02x5               g156(.a(\b[20] ), .b(\a[21] ), .o1(new_n252));
  norb02aa1n02x5               g157(.a(new_n252), .b(new_n251), .out0(new_n253));
  nanp03aa1n02x5               g158(.a(new_n233), .b(new_n230), .c(new_n237), .o1(new_n254));
  inv000aa1n02x5               g159(.a(new_n253), .o1(new_n255));
  and003aa1n02x5               g160(.a(new_n254), .b(new_n255), .c(new_n247), .o(new_n256));
  aoi022aa1n02x5               g161(.a(new_n250), .b(new_n253), .c(new_n245), .d(new_n256), .o1(\s[21] ));
  aoai13aa1n02x5               g162(.a(new_n253), .b(new_n248), .c(new_n212), .d(new_n244), .o1(new_n258));
  nor042aa1n02x5               g163(.a(\b[21] ), .b(\a[22] ), .o1(new_n259));
  nanp02aa1n02x5               g164(.a(\b[21] ), .b(\a[22] ), .o1(new_n260));
  norb02aa1n02x5               g165(.a(new_n260), .b(new_n259), .out0(new_n261));
  aoib12aa1n02x5               g166(.a(new_n251), .b(new_n260), .c(new_n259), .out0(new_n262));
  inv000aa1d42x5               g167(.a(new_n251), .o1(new_n263));
  aoai13aa1n04x5               g168(.a(new_n263), .b(new_n255), .c(new_n245), .d(new_n249), .o1(new_n264));
  aoi022aa1n03x5               g169(.a(new_n264), .b(new_n261), .c(new_n258), .d(new_n262), .o1(\s[22] ));
  norp02aa1n02x5               g170(.a(new_n241), .b(new_n246), .o1(new_n266));
  nano23aa1n06x5               g171(.a(new_n251), .b(new_n259), .c(new_n260), .d(new_n252), .out0(new_n267));
  and003aa1n02x5               g172(.a(new_n227), .b(new_n266), .c(new_n267), .o(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n220), .c(new_n176), .d(new_n204), .o1(new_n269));
  tech160nm_fiaoi012aa1n03p5x5 g174(.a(new_n259), .b(new_n251), .c(new_n260), .o1(new_n270));
  inv040aa1n03x5               g175(.a(new_n270), .o1(new_n271));
  aoi012aa1n02x5               g176(.a(new_n271), .b(new_n248), .c(new_n267), .o1(new_n272));
  nanp02aa1n02x5               g177(.a(new_n269), .b(new_n272), .o1(new_n273));
  nor042aa1n06x5               g178(.a(\b[22] ), .b(\a[23] ), .o1(new_n274));
  nand42aa1n02x5               g179(.a(\b[22] ), .b(\a[23] ), .o1(new_n275));
  norb02aa1n09x5               g180(.a(new_n275), .b(new_n274), .out0(new_n276));
  aoi112aa1n02x5               g181(.a(new_n276), .b(new_n271), .c(new_n248), .d(new_n267), .o1(new_n277));
  aoi022aa1n02x5               g182(.a(new_n273), .b(new_n276), .c(new_n269), .d(new_n277), .o1(\s[23] ));
  inv000aa1n02x5               g183(.a(new_n272), .o1(new_n279));
  aoai13aa1n02x5               g184(.a(new_n276), .b(new_n279), .c(new_n212), .d(new_n268), .o1(new_n280));
  nor022aa1n03x5               g185(.a(\b[23] ), .b(\a[24] ), .o1(new_n281));
  nand42aa1n03x5               g186(.a(\b[23] ), .b(\a[24] ), .o1(new_n282));
  norb02aa1n02x5               g187(.a(new_n282), .b(new_n281), .out0(new_n283));
  aoib12aa1n02x5               g188(.a(new_n274), .b(new_n282), .c(new_n281), .out0(new_n284));
  inv000aa1d42x5               g189(.a(new_n274), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n276), .o1(new_n286));
  aoai13aa1n06x5               g191(.a(new_n285), .b(new_n286), .c(new_n269), .d(new_n272), .o1(new_n287));
  aoi022aa1n03x5               g192(.a(new_n287), .b(new_n283), .c(new_n280), .d(new_n284), .o1(\s[24] ));
  nano23aa1n06x5               g193(.a(new_n274), .b(new_n281), .c(new_n282), .d(new_n275), .out0(new_n289));
  nand02aa1n02x5               g194(.a(new_n289), .b(new_n267), .o1(new_n290));
  nano22aa1n02x4               g195(.a(new_n290), .b(new_n227), .c(new_n266), .out0(new_n291));
  aoai13aa1n06x5               g196(.a(new_n291), .b(new_n220), .c(new_n176), .d(new_n204), .o1(new_n292));
  nano32aa1n02x4               g197(.a(new_n255), .b(new_n283), .c(new_n261), .d(new_n276), .out0(new_n293));
  nand02aa1n02x5               g198(.a(new_n248), .b(new_n293), .o1(new_n294));
  oaoi03aa1n02x5               g199(.a(\a[24] ), .b(\b[23] ), .c(new_n285), .o1(new_n295));
  aoi012aa1n06x5               g200(.a(new_n295), .b(new_n289), .c(new_n271), .o1(new_n296));
  aoai13aa1n12x5               g201(.a(new_n296), .b(new_n290), .c(new_n254), .d(new_n247), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n297), .o1(new_n298));
  xorc02aa1n02x5               g203(.a(\a[25] ), .b(\b[24] ), .out0(new_n299));
  inv000aa1d42x5               g204(.a(new_n299), .o1(new_n300));
  tech160nm_fiaoi012aa1n05x5   g205(.a(new_n300), .b(new_n292), .c(new_n298), .o1(new_n301));
  aoi112aa1n02x5               g206(.a(new_n295), .b(new_n299), .c(new_n289), .d(new_n271), .o1(new_n302));
  aoi013aa1n02x4               g207(.a(new_n301), .b(new_n294), .c(new_n292), .d(new_n302), .o1(\s[25] ));
  aoai13aa1n03x5               g208(.a(new_n299), .b(new_n297), .c(new_n212), .d(new_n291), .o1(new_n304));
  xorc02aa1n02x5               g209(.a(\a[26] ), .b(\b[25] ), .out0(new_n305));
  nor042aa1n06x5               g210(.a(\b[24] ), .b(\a[25] ), .o1(new_n306));
  norp02aa1n02x5               g211(.a(new_n305), .b(new_n306), .o1(new_n307));
  inv040aa1n03x5               g212(.a(new_n306), .o1(new_n308));
  aoai13aa1n04x5               g213(.a(new_n308), .b(new_n300), .c(new_n292), .d(new_n298), .o1(new_n309));
  aoi022aa1n02x7               g214(.a(new_n309), .b(new_n305), .c(new_n304), .d(new_n307), .o1(\s[26] ));
  nanp02aa1n02x5               g215(.a(\b[24] ), .b(\a[25] ), .o1(new_n311));
  tech160nm_fixnrc02aa1n02p5x5 g216(.a(\b[25] ), .b(\a[26] ), .out0(new_n312));
  nano22aa1n03x7               g217(.a(new_n312), .b(new_n308), .c(new_n311), .out0(new_n313));
  nano32aa1n03x7               g218(.a(new_n290), .b(new_n227), .c(new_n313), .d(new_n266), .out0(new_n314));
  aoai13aa1n06x5               g219(.a(new_n314), .b(new_n220), .c(new_n176), .d(new_n204), .o1(new_n315));
  oaoi03aa1n02x5               g220(.a(\a[26] ), .b(\b[25] ), .c(new_n308), .o1(new_n316));
  aoi012aa1n12x5               g221(.a(new_n316), .b(new_n297), .c(new_n313), .o1(new_n317));
  nanp02aa1n02x5               g222(.a(new_n315), .b(new_n317), .o1(new_n318));
  xorc02aa1n12x5               g223(.a(\a[27] ), .b(\b[26] ), .out0(new_n319));
  aoi112aa1n02x5               g224(.a(new_n319), .b(new_n316), .c(new_n297), .d(new_n313), .o1(new_n320));
  aoi022aa1n02x5               g225(.a(new_n318), .b(new_n319), .c(new_n315), .d(new_n320), .o1(\s[27] ));
  inv000aa1d42x5               g226(.a(new_n313), .o1(new_n322));
  inv000aa1n02x5               g227(.a(new_n316), .o1(new_n323));
  aoai13aa1n02x7               g228(.a(new_n323), .b(new_n322), .c(new_n294), .d(new_n296), .o1(new_n324));
  aoai13aa1n02x5               g229(.a(new_n319), .b(new_n324), .c(new_n212), .d(new_n314), .o1(new_n325));
  tech160nm_fixorc02aa1n02p5x5 g230(.a(\a[28] ), .b(\b[27] ), .out0(new_n326));
  norp02aa1n02x5               g231(.a(\b[26] ), .b(\a[27] ), .o1(new_n327));
  norp02aa1n02x5               g232(.a(new_n326), .b(new_n327), .o1(new_n328));
  inv000aa1n06x5               g233(.a(new_n327), .o1(new_n329));
  inv000aa1d42x5               g234(.a(new_n319), .o1(new_n330));
  aoai13aa1n03x5               g235(.a(new_n329), .b(new_n330), .c(new_n315), .d(new_n317), .o1(new_n331));
  aoi022aa1n03x5               g236(.a(new_n331), .b(new_n326), .c(new_n325), .d(new_n328), .o1(\s[28] ));
  and002aa1n02x5               g237(.a(new_n326), .b(new_n319), .o(new_n333));
  aoai13aa1n02x5               g238(.a(new_n333), .b(new_n324), .c(new_n212), .d(new_n314), .o1(new_n334));
  inv000aa1d42x5               g239(.a(new_n333), .o1(new_n335));
  oaoi03aa1n12x5               g240(.a(\a[28] ), .b(\b[27] ), .c(new_n329), .o1(new_n336));
  inv000aa1d42x5               g241(.a(new_n336), .o1(new_n337));
  aoai13aa1n03x5               g242(.a(new_n337), .b(new_n335), .c(new_n315), .d(new_n317), .o1(new_n338));
  tech160nm_fixorc02aa1n02p5x5 g243(.a(\a[29] ), .b(\b[28] ), .out0(new_n339));
  norp02aa1n02x5               g244(.a(new_n336), .b(new_n339), .o1(new_n340));
  aoi022aa1n03x5               g245(.a(new_n338), .b(new_n339), .c(new_n334), .d(new_n340), .o1(\s[29] ));
  nanp02aa1n02x5               g246(.a(\b[0] ), .b(\a[1] ), .o1(new_n342));
  xorb03aa1n02x5               g247(.a(new_n342), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x5               g248(.a(new_n330), .b(new_n326), .c(new_n339), .out0(new_n344));
  aoai13aa1n03x5               g249(.a(new_n344), .b(new_n324), .c(new_n212), .d(new_n314), .o1(new_n345));
  inv000aa1d42x5               g250(.a(new_n344), .o1(new_n346));
  inv000aa1d42x5               g251(.a(\b[28] ), .o1(new_n347));
  oaib12aa1n09x5               g252(.a(new_n336), .b(new_n347), .c(\a[29] ), .out0(new_n348));
  oa0012aa1n02x5               g253(.a(new_n348), .b(\b[28] ), .c(\a[29] ), .o(new_n349));
  aoai13aa1n03x5               g254(.a(new_n349), .b(new_n346), .c(new_n315), .d(new_n317), .o1(new_n350));
  xorc02aa1n02x5               g255(.a(\a[30] ), .b(\b[29] ), .out0(new_n351));
  oabi12aa1n02x5               g256(.a(new_n351), .b(\a[29] ), .c(\b[28] ), .out0(new_n352));
  norb02aa1n02x5               g257(.a(new_n348), .b(new_n352), .out0(new_n353));
  aoi022aa1n03x5               g258(.a(new_n350), .b(new_n351), .c(new_n345), .d(new_n353), .o1(\s[30] ));
  nano32aa1n02x5               g259(.a(new_n330), .b(new_n351), .c(new_n326), .d(new_n339), .out0(new_n355));
  aoai13aa1n02x5               g260(.a(new_n355), .b(new_n324), .c(new_n212), .d(new_n314), .o1(new_n356));
  xorc02aa1n02x5               g261(.a(\a[31] ), .b(\b[30] ), .out0(new_n357));
  oai122aa1n02x7               g262(.a(new_n348), .b(\a[30] ), .c(\b[29] ), .d(\a[29] ), .e(\b[28] ), .o1(new_n358));
  aob012aa1n02x5               g263(.a(new_n358), .b(\b[29] ), .c(\a[30] ), .out0(new_n359));
  norb02aa1n02x5               g264(.a(new_n359), .b(new_n357), .out0(new_n360));
  inv000aa1d42x5               g265(.a(new_n355), .o1(new_n361));
  aoai13aa1n03x5               g266(.a(new_n359), .b(new_n361), .c(new_n315), .d(new_n317), .o1(new_n362));
  aoi022aa1n03x5               g267(.a(new_n362), .b(new_n357), .c(new_n356), .d(new_n360), .o1(\s[31] ));
  xobna2aa1n03x5               g268(.a(new_n165), .b(new_n99), .c(new_n98), .out0(\s[3] ));
  aoi013aa1n02x4               g269(.a(new_n100), .b(new_n98), .c(new_n99), .d(new_n101), .o1(new_n365));
  aoai13aa1n02x5               g270(.a(new_n107), .b(new_n365), .c(new_n104), .d(new_n103), .o1(\s[4] ));
  inv000aa1d42x5               g271(.a(\a[5] ), .o1(new_n367));
  inv000aa1d42x5               g272(.a(\b[4] ), .o1(new_n368));
  nanp02aa1n02x5               g273(.a(new_n368), .b(new_n367), .o1(new_n369));
  nanp02aa1n02x5               g274(.a(\b[4] ), .b(\a[5] ), .o1(new_n370));
  aoi022aa1n02x5               g275(.a(new_n107), .b(new_n103), .c(new_n369), .d(new_n370), .o1(new_n371));
  aoi013aa1n02x4               g276(.a(new_n371), .b(new_n369), .c(new_n111), .d(new_n107), .o1(\s[5] ));
  nand23aa1n03x5               g277(.a(new_n107), .b(new_n111), .c(new_n369), .o1(new_n373));
  xorc02aa1n02x5               g278(.a(\a[6] ), .b(\b[5] ), .out0(new_n374));
  nand22aa1n03x5               g279(.a(new_n373), .b(new_n120), .o1(new_n375));
  aoai13aa1n02x5               g280(.a(new_n375), .b(new_n374), .c(new_n369), .d(new_n373), .o1(\s[6] ));
  nona23aa1n06x5               g281(.a(new_n375), .b(new_n167), .c(new_n121), .d(new_n119), .out0(new_n377));
  aboi22aa1n03x5               g282(.a(new_n121), .b(new_n167), .c(new_n375), .d(new_n117), .out0(new_n378));
  norb02aa1n02x5               g283(.a(new_n377), .b(new_n378), .out0(\s[7] ));
  nanb02aa1n02x5               g284(.a(\b[6] ), .b(new_n108), .out0(new_n380));
  xorc02aa1n02x5               g285(.a(\a[8] ), .b(\b[7] ), .out0(new_n381));
  xnbna2aa1n03x5               g286(.a(new_n381), .b(new_n377), .c(new_n380), .out0(\s[8] ));
  nanp02aa1n02x5               g287(.a(new_n116), .b(new_n107), .o1(new_n383));
  aoi112aa1n02x5               g288(.a(new_n174), .b(new_n124), .c(new_n173), .d(new_n172), .o1(new_n384));
  aoi022aa1n02x5               g289(.a(new_n176), .b(new_n124), .c(new_n383), .d(new_n384), .o1(\s[9] ));
endmodule


