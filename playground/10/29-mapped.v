// Benchmark "adder" written by ABC on Wed Jul 17 17:19:40 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n159, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n287, new_n288,
    new_n289, new_n290, new_n291, new_n292, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n350, new_n352,
    new_n353, new_n355, new_n356, new_n358, new_n359, new_n360, new_n362,
    new_n363, new_n365, new_n366, new_n367, new_n369, new_n370;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand02aa1d08x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nanp02aa1n04x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  norb03aa1n03x5               g004(.a(new_n98), .b(new_n97), .c(new_n99), .out0(new_n100));
  inv000aa1n02x5               g005(.a(new_n100), .o1(new_n101));
  tech160nm_fioai012aa1n04x5   g006(.a(new_n98), .b(\b[3] ), .c(\a[4] ), .o1(new_n102));
  nand42aa1n03x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand42aa1n03x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n04x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanb03aa1n06x5               g010(.a(new_n105), .b(new_n103), .c(new_n104), .out0(new_n106));
  nona22aa1n09x5               g011(.a(new_n101), .b(new_n106), .c(new_n102), .out0(new_n107));
  inv000aa1d42x5               g012(.a(\a[4] ), .o1(new_n108));
  inv000aa1d42x5               g013(.a(\b[3] ), .o1(new_n109));
  oaoi03aa1n02x5               g014(.a(new_n108), .b(new_n109), .c(new_n105), .o1(new_n110));
  oai022aa1d18x5               g015(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n111));
  aoi022aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n112));
  xorc02aa1n12x5               g017(.a(\a[8] ), .b(\b[7] ), .out0(new_n113));
  nand42aa1n06x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  oai012aa1n12x5               g019(.a(new_n114), .b(\b[6] ), .c(\a[7] ), .o1(new_n115));
  nona23aa1n03x5               g020(.a(new_n113), .b(new_n112), .c(new_n115), .d(new_n111), .out0(new_n116));
  orn002aa1n02x5               g021(.a(\a[7] ), .b(\b[6] ), .o(new_n117));
  oaoi03aa1n03x5               g022(.a(\a[8] ), .b(\b[7] ), .c(new_n117), .o1(new_n118));
  and002aa1n02x7               g023(.a(\b[6] ), .b(\a[7] ), .o(new_n119));
  nor022aa1n04x5               g024(.a(new_n115), .b(new_n119), .o1(new_n120));
  aoi013aa1n09x5               g025(.a(new_n118), .b(new_n120), .c(new_n113), .d(new_n111), .o1(new_n121));
  aoai13aa1n06x5               g026(.a(new_n121), .b(new_n116), .c(new_n107), .d(new_n110), .o1(new_n122));
  nor042aa1n04x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  xnrc02aa1n12x5               g028(.a(\b[8] ), .b(\a[9] ), .out0(new_n124));
  inv000aa1d42x5               g029(.a(new_n124), .o1(new_n125));
  xnrc02aa1n12x5               g030(.a(\b[9] ), .b(\a[10] ), .out0(new_n126));
  inv000aa1d42x5               g031(.a(new_n126), .o1(new_n127));
  aoi112aa1n02x5               g032(.a(new_n123), .b(new_n127), .c(new_n122), .d(new_n125), .o1(new_n128));
  aoai13aa1n09x5               g033(.a(new_n127), .b(new_n123), .c(new_n122), .d(new_n125), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(\s[10] ));
  inv000aa1d42x5               g035(.a(\a[10] ), .o1(new_n131));
  inv000aa1d42x5               g036(.a(\b[9] ), .o1(new_n132));
  oao003aa1n06x5               g037(.a(new_n131), .b(new_n132), .c(new_n123), .carry(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  tech160nm_fixnrc02aa1n04x5   g039(.a(\b[10] ), .b(\a[11] ), .out0(new_n135));
  inv000aa1n04x5               g040(.a(new_n135), .o1(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n136), .b(new_n129), .c(new_n134), .out0(\s[11] ));
  aob012aa1n02x5               g042(.a(new_n136), .b(new_n129), .c(new_n134), .out0(new_n138));
  tech160nm_fixnrc02aa1n05x5   g043(.a(\b[11] ), .b(\a[12] ), .out0(new_n139));
  inv030aa1n03x5               g044(.a(new_n139), .o1(new_n140));
  nor042aa1n03x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n139), .b(new_n141), .out0(new_n142));
  inv000aa1d42x5               g047(.a(new_n141), .o1(new_n143));
  aoai13aa1n02x5               g048(.a(new_n143), .b(new_n135), .c(new_n129), .d(new_n134), .o1(new_n144));
  aoi022aa1n02x5               g049(.a(new_n144), .b(new_n140), .c(new_n138), .d(new_n142), .o1(\s[12] ));
  oai013aa1n09x5               g050(.a(new_n110), .b(new_n100), .c(new_n106), .d(new_n102), .o1(new_n146));
  inv030aa1n02x5               g051(.a(new_n116), .o1(new_n147));
  inv000aa1d42x5               g052(.a(new_n121), .o1(new_n148));
  nona23aa1d24x5               g053(.a(new_n136), .b(new_n140), .c(new_n126), .d(new_n124), .out0(new_n149));
  inv000aa1d42x5               g054(.a(new_n149), .o1(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n148), .c(new_n147), .d(new_n146), .o1(new_n151));
  nona22aa1n03x5               g056(.a(new_n133), .b(new_n135), .c(new_n139), .out0(new_n152));
  oao003aa1n02x5               g057(.a(\a[12] ), .b(\b[11] ), .c(new_n143), .carry(new_n153));
  nanp02aa1n02x5               g058(.a(new_n152), .b(new_n153), .o1(new_n154));
  nanb02aa1n02x5               g059(.a(new_n154), .b(new_n151), .out0(new_n155));
  xorc02aa1n02x5               g060(.a(\a[13] ), .b(\b[12] ), .out0(new_n156));
  nano22aa1n02x4               g061(.a(new_n156), .b(new_n152), .c(new_n153), .out0(new_n157));
  aoi022aa1n02x5               g062(.a(new_n155), .b(new_n156), .c(new_n151), .d(new_n157), .o1(\s[13] ));
  inv000aa1d42x5               g063(.a(\a[13] ), .o1(new_n159));
  nanb02aa1n12x5               g064(.a(\b[12] ), .b(new_n159), .out0(new_n160));
  aoai13aa1n02x5               g065(.a(new_n156), .b(new_n154), .c(new_n122), .d(new_n150), .o1(new_n161));
  xorc02aa1n02x5               g066(.a(\a[14] ), .b(\b[13] ), .out0(new_n162));
  xnbna2aa1n03x5               g067(.a(new_n162), .b(new_n161), .c(new_n160), .out0(\s[14] ));
  inv000aa1d42x5               g068(.a(\a[14] ), .o1(new_n164));
  xroi22aa1d06x4               g069(.a(new_n159), .b(\b[12] ), .c(new_n164), .d(\b[13] ), .out0(new_n165));
  aoai13aa1n02x5               g070(.a(new_n165), .b(new_n154), .c(new_n122), .d(new_n150), .o1(new_n166));
  oaoi03aa1n09x5               g071(.a(\a[14] ), .b(\b[13] ), .c(new_n160), .o1(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  xorc02aa1n06x5               g073(.a(\a[15] ), .b(\b[14] ), .out0(new_n169));
  xnbna2aa1n03x5               g074(.a(new_n169), .b(new_n166), .c(new_n168), .out0(\s[15] ));
  aoai13aa1n02x5               g075(.a(new_n169), .b(new_n167), .c(new_n155), .d(new_n165), .o1(new_n171));
  xorc02aa1n02x5               g076(.a(\a[16] ), .b(\b[15] ), .out0(new_n172));
  norp02aa1n12x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  norp02aa1n02x5               g078(.a(new_n172), .b(new_n173), .o1(new_n174));
  inv000aa1n03x5               g079(.a(new_n173), .o1(new_n175));
  inv000aa1d42x5               g080(.a(new_n169), .o1(new_n176));
  aoai13aa1n02x5               g081(.a(new_n175), .b(new_n176), .c(new_n166), .d(new_n168), .o1(new_n177));
  aoi022aa1n02x5               g082(.a(new_n177), .b(new_n172), .c(new_n171), .d(new_n174), .o1(\s[16] ));
  xorc02aa1n02x5               g083(.a(\a[17] ), .b(\b[16] ), .out0(new_n179));
  nanp02aa1n02x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  xnrc02aa1n02x5               g085(.a(\b[15] ), .b(\a[16] ), .out0(new_n181));
  nano22aa1n03x7               g086(.a(new_n181), .b(new_n180), .c(new_n175), .out0(new_n182));
  nano22aa1d15x5               g087(.a(new_n149), .b(new_n165), .c(new_n182), .out0(new_n183));
  aoai13aa1n12x5               g088(.a(new_n183), .b(new_n148), .c(new_n146), .d(new_n147), .o1(new_n184));
  inv000aa1d42x5               g089(.a(new_n184), .o1(new_n185));
  nand02aa1d04x5               g090(.a(new_n165), .b(new_n182), .o1(new_n186));
  oaoi03aa1n02x5               g091(.a(\a[16] ), .b(\b[15] ), .c(new_n175), .o1(new_n187));
  aoi013aa1n06x5               g092(.a(new_n187), .b(new_n167), .c(new_n169), .d(new_n172), .o1(new_n188));
  aoai13aa1n06x5               g093(.a(new_n188), .b(new_n186), .c(new_n152), .d(new_n153), .o1(new_n189));
  inv040aa1n03x5               g094(.a(new_n189), .o1(new_n190));
  nanp02aa1n12x5               g095(.a(new_n184), .b(new_n190), .o1(new_n191));
  inv000aa1d42x5               g096(.a(new_n186), .o1(new_n192));
  nanp02aa1n02x5               g097(.a(new_n192), .b(new_n154), .o1(new_n193));
  nano22aa1n02x4               g098(.a(new_n168), .b(new_n169), .c(new_n172), .out0(new_n194));
  aoi112aa1n02x5               g099(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n195));
  inv000aa1d42x5               g100(.a(\a[17] ), .o1(new_n196));
  inv000aa1d42x5               g101(.a(\b[16] ), .o1(new_n197));
  oai022aa1n02x5               g102(.a(\a[16] ), .b(\b[15] ), .c(\b[16] ), .d(\a[17] ), .o1(new_n198));
  oabi12aa1n02x5               g103(.a(new_n198), .b(new_n196), .c(new_n197), .out0(new_n199));
  nona32aa1n02x4               g104(.a(new_n193), .b(new_n199), .c(new_n195), .d(new_n194), .out0(new_n200));
  obai22aa1n02x7               g105(.a(new_n191), .b(new_n179), .c(new_n200), .d(new_n185), .out0(\s[17] ));
  xnrc02aa1n02x5               g106(.a(\b[17] ), .b(\a[18] ), .out0(new_n202));
  nona32aa1n02x4               g107(.a(new_n193), .b(new_n198), .c(new_n195), .d(new_n194), .out0(new_n203));
  nanb02aa1n02x5               g108(.a(new_n203), .b(new_n184), .out0(new_n204));
  obai22aa1n02x5               g109(.a(new_n184), .b(new_n203), .c(new_n196), .d(new_n197), .out0(new_n205));
  oai022aa1n02x5               g110(.a(new_n196), .b(new_n197), .c(\b[17] ), .d(\a[18] ), .o1(new_n206));
  aoi012aa1n02x5               g111(.a(new_n206), .b(\a[18] ), .c(\b[17] ), .o1(new_n207));
  aoi022aa1n02x5               g112(.a(new_n205), .b(new_n202), .c(new_n204), .d(new_n207), .o1(\s[18] ));
  inv000aa1d42x5               g113(.a(\a[18] ), .o1(new_n209));
  xroi22aa1d04x5               g114(.a(new_n196), .b(\b[16] ), .c(new_n209), .d(\b[17] ), .out0(new_n210));
  aoai13aa1n06x5               g115(.a(new_n210), .b(new_n189), .c(new_n122), .d(new_n183), .o1(new_n211));
  aoi112aa1n03x4               g116(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n212));
  aoib12aa1n06x5               g117(.a(new_n212), .b(new_n209), .c(\b[17] ), .out0(new_n213));
  nand22aa1n06x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  nor002aa1d32x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  norb02aa1n02x5               g120(.a(new_n214), .b(new_n215), .out0(new_n216));
  xnbna2aa1n03x5               g121(.a(new_n216), .b(new_n211), .c(new_n213), .out0(\s[19] ));
  xnrc02aa1n02x5               g122(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n02x5               g123(.a(\b[16] ), .b(\a[17] ), .o1(new_n219));
  aob012aa1n02x5               g124(.a(new_n219), .b(\b[17] ), .c(\a[18] ), .out0(new_n220));
  oaib12aa1n06x5               g125(.a(new_n220), .b(\b[17] ), .c(new_n209), .out0(new_n221));
  aoai13aa1n03x5               g126(.a(new_n216), .b(new_n221), .c(new_n191), .d(new_n210), .o1(new_n222));
  nor002aa1n06x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  nand02aa1d06x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n224), .b(new_n223), .out0(new_n225));
  inv000aa1d42x5               g130(.a(\a[19] ), .o1(new_n226));
  inv000aa1d42x5               g131(.a(\b[18] ), .o1(new_n227));
  aboi22aa1n03x5               g132(.a(new_n223), .b(new_n224), .c(new_n226), .d(new_n227), .out0(new_n228));
  inv040aa1n09x5               g133(.a(new_n215), .o1(new_n229));
  inv000aa1d42x5               g134(.a(new_n216), .o1(new_n230));
  aoai13aa1n02x5               g135(.a(new_n229), .b(new_n230), .c(new_n211), .d(new_n213), .o1(new_n231));
  aoi022aa1n02x7               g136(.a(new_n231), .b(new_n225), .c(new_n222), .d(new_n228), .o1(\s[20] ));
  nona23aa1n09x5               g137(.a(new_n214), .b(new_n224), .c(new_n223), .d(new_n215), .out0(new_n233));
  norb03aa1n12x5               g138(.a(new_n179), .b(new_n233), .c(new_n202), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  tech160nm_fiaoi012aa1n05x5   g140(.a(new_n235), .b(new_n184), .c(new_n190), .o1(new_n236));
  oaoi03aa1n12x5               g141(.a(\a[20] ), .b(\b[19] ), .c(new_n229), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  oai012aa1n18x5               g143(.a(new_n238), .b(new_n233), .c(new_n213), .o1(new_n239));
  tech160nm_fixnrc02aa1n04x5   g144(.a(\b[20] ), .b(\a[21] ), .out0(new_n240));
  oabi12aa1n06x5               g145(.a(new_n240), .b(new_n236), .c(new_n239), .out0(new_n241));
  oai112aa1n02x5               g146(.a(new_n238), .b(new_n240), .c(new_n233), .d(new_n213), .o1(new_n242));
  oa0012aa1n03x5               g147(.a(new_n241), .b(new_n236), .c(new_n242), .o(\s[21] ));
  xnrc02aa1n12x5               g148(.a(\b[21] ), .b(\a[22] ), .out0(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  nor042aa1n06x5               g150(.a(\b[20] ), .b(\a[21] ), .o1(new_n246));
  norb02aa1n02x5               g151(.a(new_n244), .b(new_n246), .out0(new_n247));
  aoai13aa1n02x5               g152(.a(new_n234), .b(new_n189), .c(new_n122), .d(new_n183), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n239), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n246), .o1(new_n250));
  aoai13aa1n02x5               g155(.a(new_n250), .b(new_n240), .c(new_n248), .d(new_n249), .o1(new_n251));
  aoi022aa1n03x5               g156(.a(new_n251), .b(new_n245), .c(new_n241), .d(new_n247), .o1(\s[22] ));
  nano23aa1n06x5               g157(.a(new_n223), .b(new_n215), .c(new_n224), .d(new_n214), .out0(new_n253));
  nor042aa1n04x5               g158(.a(new_n244), .b(new_n240), .o1(new_n254));
  and003aa1n02x5               g159(.a(new_n210), .b(new_n254), .c(new_n253), .o(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n189), .c(new_n122), .d(new_n183), .o1(new_n256));
  aoai13aa1n12x5               g161(.a(new_n254), .b(new_n237), .c(new_n253), .d(new_n221), .o1(new_n257));
  oaoi03aa1n12x5               g162(.a(\a[22] ), .b(\b[21] ), .c(new_n250), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  nanp02aa1n02x5               g164(.a(new_n257), .b(new_n259), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  nor002aa1n02x5               g166(.a(\b[22] ), .b(\a[23] ), .o1(new_n262));
  nanp02aa1n02x5               g167(.a(\b[22] ), .b(\a[23] ), .o1(new_n263));
  norb02aa1n02x5               g168(.a(new_n263), .b(new_n262), .out0(new_n264));
  inv000aa1d42x5               g169(.a(new_n257), .o1(new_n265));
  nona23aa1n02x4               g170(.a(new_n256), .b(new_n264), .c(new_n258), .d(new_n265), .out0(new_n266));
  aoai13aa1n02x5               g171(.a(new_n266), .b(new_n264), .c(new_n256), .d(new_n261), .o1(\s[23] ));
  nor002aa1n02x5               g172(.a(\b[23] ), .b(\a[24] ), .o1(new_n268));
  nanp02aa1n02x5               g173(.a(\b[23] ), .b(\a[24] ), .o1(new_n269));
  nanb02aa1n02x5               g174(.a(new_n268), .b(new_n269), .out0(new_n270));
  nona32aa1n02x4               g175(.a(new_n256), .b(new_n262), .c(new_n258), .d(new_n265), .out0(new_n271));
  nanp02aa1n02x5               g176(.a(new_n271), .b(new_n263), .o1(new_n272));
  nano22aa1n02x4               g177(.a(new_n268), .b(new_n263), .c(new_n269), .out0(new_n273));
  aoi022aa1n02x5               g178(.a(new_n272), .b(new_n270), .c(new_n271), .d(new_n273), .o1(\s[24] ));
  nano23aa1d12x5               g179(.a(new_n262), .b(new_n268), .c(new_n269), .d(new_n263), .out0(new_n275));
  inv040aa1n03x5               g180(.a(new_n275), .o1(new_n276));
  nano32aa1n02x4               g181(.a(new_n276), .b(new_n210), .c(new_n254), .d(new_n253), .out0(new_n277));
  aoai13aa1n02x5               g182(.a(new_n277), .b(new_n189), .c(new_n122), .d(new_n183), .o1(new_n278));
  oai012aa1n02x5               g183(.a(new_n269), .b(new_n268), .c(new_n262), .o1(new_n279));
  aoai13aa1n12x5               g184(.a(new_n279), .b(new_n276), .c(new_n257), .d(new_n259), .o1(new_n280));
  xorc02aa1n12x5               g185(.a(\a[25] ), .b(\b[24] ), .out0(new_n281));
  aoai13aa1n06x5               g186(.a(new_n281), .b(new_n280), .c(new_n191), .d(new_n277), .o1(new_n282));
  aoai13aa1n02x5               g187(.a(new_n275), .b(new_n258), .c(new_n239), .d(new_n254), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n281), .o1(new_n284));
  and003aa1n02x5               g189(.a(new_n283), .b(new_n284), .c(new_n279), .o(new_n285));
  aobi12aa1n03x7               g190(.a(new_n282), .b(new_n285), .c(new_n278), .out0(\s[25] ));
  xorc02aa1n02x5               g191(.a(\a[26] ), .b(\b[25] ), .out0(new_n287));
  norp02aa1n02x5               g192(.a(\b[24] ), .b(\a[25] ), .o1(new_n288));
  norp02aa1n02x5               g193(.a(new_n287), .b(new_n288), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n280), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n288), .o1(new_n291));
  aoai13aa1n02x5               g196(.a(new_n291), .b(new_n284), .c(new_n278), .d(new_n290), .o1(new_n292));
  aoi022aa1n03x5               g197(.a(new_n292), .b(new_n287), .c(new_n282), .d(new_n289), .o1(\s[26] ));
  and002aa1n06x5               g198(.a(new_n287), .b(new_n281), .o(new_n294));
  nano32aa1n03x7               g199(.a(new_n235), .b(new_n294), .c(new_n254), .d(new_n275), .out0(new_n295));
  aoai13aa1n06x5               g200(.a(new_n295), .b(new_n189), .c(new_n122), .d(new_n183), .o1(new_n296));
  nanp02aa1n02x5               g201(.a(\b[25] ), .b(\a[26] ), .o1(new_n297));
  oai022aa1n02x5               g202(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n298));
  aoi022aa1n09x5               g203(.a(new_n280), .b(new_n294), .c(new_n297), .d(new_n298), .o1(new_n299));
  xorc02aa1n02x5               g204(.a(\a[27] ), .b(\b[26] ), .out0(new_n300));
  nanp02aa1n06x5               g205(.a(new_n280), .b(new_n294), .o1(new_n301));
  nanp02aa1n02x5               g206(.a(new_n298), .b(new_n297), .o1(new_n302));
  and002aa1n02x5               g207(.a(new_n300), .b(new_n302), .o(new_n303));
  nanp03aa1n02x5               g208(.a(new_n296), .b(new_n301), .c(new_n303), .o1(new_n304));
  aoai13aa1n02x5               g209(.a(new_n304), .b(new_n300), .c(new_n296), .d(new_n299), .o1(\s[27] ));
  xnrc02aa1n02x5               g210(.a(\b[27] ), .b(\a[28] ), .out0(new_n306));
  inv000aa1d42x5               g211(.a(\a[27] ), .o1(new_n307));
  inv000aa1d42x5               g212(.a(\b[26] ), .o1(new_n308));
  aoi022aa1n02x5               g213(.a(new_n298), .b(new_n297), .c(new_n307), .d(new_n308), .o1(new_n309));
  nanp03aa1n03x5               g214(.a(new_n296), .b(new_n301), .c(new_n309), .o1(new_n310));
  oaib12aa1n06x5               g215(.a(new_n310), .b(new_n308), .c(\a[27] ), .out0(new_n311));
  oai022aa1n02x5               g216(.a(new_n307), .b(new_n308), .c(\b[27] ), .d(\a[28] ), .o1(new_n312));
  aoi012aa1n02x5               g217(.a(new_n312), .b(\a[28] ), .c(\b[27] ), .o1(new_n313));
  aoi022aa1n02x7               g218(.a(new_n311), .b(new_n306), .c(new_n310), .d(new_n313), .o1(\s[28] ));
  inv000aa1d42x5               g219(.a(new_n294), .o1(new_n315));
  aoai13aa1n02x5               g220(.a(new_n302), .b(new_n315), .c(new_n283), .d(new_n279), .o1(new_n316));
  inv000aa1d42x5               g221(.a(\a[28] ), .o1(new_n317));
  xroi22aa1d06x4               g222(.a(new_n307), .b(\b[26] ), .c(new_n317), .d(\b[27] ), .out0(new_n318));
  aoai13aa1n03x5               g223(.a(new_n318), .b(new_n316), .c(new_n191), .d(new_n295), .o1(new_n319));
  inv000aa1d42x5               g224(.a(new_n318), .o1(new_n320));
  nanp02aa1n02x5               g225(.a(new_n308), .b(new_n307), .o1(new_n321));
  oao003aa1n02x5               g226(.a(\a[28] ), .b(\b[27] ), .c(new_n321), .carry(new_n322));
  aoai13aa1n02x7               g227(.a(new_n322), .b(new_n320), .c(new_n299), .d(new_n296), .o1(new_n323));
  xorc02aa1n02x5               g228(.a(\a[29] ), .b(\b[28] ), .out0(new_n324));
  norb02aa1n02x5               g229(.a(new_n322), .b(new_n324), .out0(new_n325));
  aoi022aa1n03x5               g230(.a(new_n323), .b(new_n324), .c(new_n319), .d(new_n325), .o1(\s[29] ));
  xorb03aa1n02x5               g231(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g232(.a(new_n306), .b(new_n300), .c(new_n324), .out0(new_n328));
  aoai13aa1n03x5               g233(.a(new_n328), .b(new_n316), .c(new_n191), .d(new_n295), .o1(new_n329));
  inv000aa1d42x5               g234(.a(new_n328), .o1(new_n330));
  oaoi03aa1n02x5               g235(.a(\a[29] ), .b(\b[28] ), .c(new_n322), .o1(new_n331));
  inv000aa1d42x5               g236(.a(new_n331), .o1(new_n332));
  aoai13aa1n02x7               g237(.a(new_n332), .b(new_n330), .c(new_n299), .d(new_n296), .o1(new_n333));
  xorc02aa1n02x5               g238(.a(\a[30] ), .b(\b[29] ), .out0(new_n334));
  aoi012aa1n02x5               g239(.a(new_n322), .b(\a[29] ), .c(\b[28] ), .o1(new_n335));
  oabi12aa1n02x5               g240(.a(new_n334), .b(\a[29] ), .c(\b[28] ), .out0(new_n336));
  norp02aa1n02x5               g241(.a(new_n335), .b(new_n336), .o1(new_n337));
  aoi022aa1n02x7               g242(.a(new_n333), .b(new_n334), .c(new_n329), .d(new_n337), .o1(\s[30] ));
  nano22aa1n06x5               g243(.a(new_n320), .b(new_n324), .c(new_n334), .out0(new_n339));
  aoai13aa1n03x5               g244(.a(new_n339), .b(new_n316), .c(new_n191), .d(new_n295), .o1(new_n340));
  xorc02aa1n02x5               g245(.a(\a[31] ), .b(\b[30] ), .out0(new_n341));
  inv000aa1d42x5               g246(.a(\a[30] ), .o1(new_n342));
  inv000aa1d42x5               g247(.a(\b[29] ), .o1(new_n343));
  oabi12aa1n02x5               g248(.a(new_n341), .b(\a[30] ), .c(\b[29] ), .out0(new_n344));
  oaoi13aa1n02x5               g249(.a(new_n344), .b(new_n331), .c(new_n342), .d(new_n343), .o1(new_n345));
  inv000aa1d42x5               g250(.a(new_n339), .o1(new_n346));
  oaoi03aa1n02x5               g251(.a(new_n342), .b(new_n343), .c(new_n331), .o1(new_n347));
  aoai13aa1n02x7               g252(.a(new_n347), .b(new_n346), .c(new_n299), .d(new_n296), .o1(new_n348));
  aoi022aa1n03x5               g253(.a(new_n348), .b(new_n341), .c(new_n340), .d(new_n345), .o1(\s[31] ));
  norb02aa1n02x5               g254(.a(new_n103), .b(new_n105), .out0(new_n350));
  xobna2aa1n03x5               g255(.a(new_n350), .b(new_n101), .c(new_n98), .out0(\s[3] ));
  aoai13aa1n02x5               g256(.a(new_n350), .b(new_n100), .c(\a[2] ), .d(\b[1] ), .o1(new_n352));
  xnrc02aa1n02x5               g257(.a(\b[3] ), .b(\a[4] ), .out0(new_n353));
  xnbna2aa1n03x5               g258(.a(new_n353), .b(new_n352), .c(new_n103), .out0(\s[4] ));
  xnrc02aa1n02x5               g259(.a(\b[4] ), .b(\a[5] ), .out0(new_n355));
  nanb03aa1n02x5               g260(.a(new_n355), .b(new_n107), .c(new_n110), .out0(new_n356));
  aob012aa1n02x5               g261(.a(new_n356), .b(new_n146), .c(new_n355), .out0(\s[5] ));
  nanp02aa1n02x5               g262(.a(\b[4] ), .b(\a[5] ), .o1(new_n358));
  xorc02aa1n02x5               g263(.a(\a[6] ), .b(\b[5] ), .out0(new_n359));
  inv000aa1d42x5               g264(.a(new_n359), .o1(new_n360));
  xnbna2aa1n03x5               g265(.a(new_n360), .b(new_n356), .c(new_n358), .out0(\s[6] ));
  aob012aa1n02x5               g266(.a(new_n359), .b(new_n356), .c(new_n358), .out0(new_n362));
  xorc02aa1n02x5               g267(.a(\a[7] ), .b(\b[6] ), .out0(new_n363));
  xobna2aa1n03x5               g268(.a(new_n363), .b(new_n362), .c(new_n114), .out0(\s[7] ));
  aoai13aa1n02x5               g269(.a(new_n114), .b(new_n360), .c(new_n356), .d(new_n358), .o1(new_n365));
  aoai13aa1n02x5               g270(.a(new_n113), .b(new_n119), .c(new_n365), .d(new_n117), .o1(new_n366));
  orn002aa1n02x5               g271(.a(new_n113), .b(new_n119), .o(new_n367));
  aoai13aa1n02x5               g272(.a(new_n366), .b(new_n367), .c(new_n363), .d(new_n365), .o1(\s[8] ));
  nanp02aa1n02x5               g273(.a(new_n146), .b(new_n147), .o1(new_n369));
  aoi113aa1n02x5               g274(.a(new_n118), .b(new_n125), .c(new_n120), .d(new_n113), .e(new_n111), .o1(new_n370));
  aoi022aa1n02x5               g275(.a(new_n122), .b(new_n125), .c(new_n369), .d(new_n370), .o1(\s[9] ));
endmodule


