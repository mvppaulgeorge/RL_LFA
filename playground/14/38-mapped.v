// Benchmark "adder" written by ABC on Wed Jul 17 19:28:51 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n336, new_n337, new_n339,
    new_n340, new_n341, new_n342, new_n344, new_n345, new_n347, new_n349,
    new_n350;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nand42aa1n10x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  aob012aa1d18x5               g003(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(new_n99));
  oai012aa1d24x5               g004(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .o1(new_n100));
  tech160nm_finor002aa1n05x5   g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nand42aa1n06x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  norb02aa1n09x5               g007(.a(new_n102), .b(new_n101), .out0(new_n103));
  nand02aa1d12x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  orn002aa1n24x5               g009(.a(\a[4] ), .b(\b[3] ), .o(new_n105));
  oai112aa1n06x5               g010(.a(new_n105), .b(new_n104), .c(\b[2] ), .d(\a[3] ), .o1(new_n106));
  aoi012aa1d24x5               g011(.a(new_n106), .b(new_n100), .c(new_n103), .o1(new_n107));
  nand02aa1n06x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  nanp02aa1n04x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  nanp02aa1n04x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nanp03aa1n02x5               g015(.a(new_n109), .b(new_n108), .c(new_n110), .o1(new_n111));
  oai012aa1n02x5               g016(.a(new_n104), .b(\b[6] ), .c(\a[7] ), .o1(new_n112));
  oai022aa1d18x5               g017(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n113));
  nor002aa1n06x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  aoi112aa1n03x5               g019(.a(new_n113), .b(new_n114), .c(\a[6] ), .d(\b[5] ), .o1(new_n115));
  nona22aa1n09x5               g020(.a(new_n115), .b(new_n111), .c(new_n112), .out0(new_n116));
  nor042aa1n03x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  oa0012aa1n02x5               g022(.a(new_n110), .b(new_n117), .c(new_n114), .o(new_n118));
  orn002aa1n03x5               g023(.a(\a[6] ), .b(\b[5] ), .o(new_n119));
  nand42aa1n04x5               g024(.a(\b[5] ), .b(\a[6] ), .o1(new_n120));
  oai112aa1n04x5               g025(.a(new_n119), .b(new_n120), .c(\b[4] ), .d(\a[5] ), .o1(new_n121));
  nano22aa1n03x7               g026(.a(new_n117), .b(new_n109), .c(new_n120), .out0(new_n122));
  norb02aa1n03x5               g027(.a(new_n110), .b(new_n114), .out0(new_n123));
  aoi013aa1n06x4               g028(.a(new_n118), .b(new_n122), .c(new_n121), .d(new_n123), .o1(new_n124));
  oai012aa1d24x5               g029(.a(new_n124), .b(new_n107), .c(new_n116), .o1(new_n125));
  xorc02aa1n06x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  nor042aa1n04x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand42aa1n03x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb02aa1n03x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  aoai13aa1n06x5               g034(.a(new_n129), .b(new_n97), .c(new_n125), .d(new_n126), .o1(new_n130));
  aoi112aa1n02x5               g035(.a(new_n129), .b(new_n97), .c(new_n125), .d(new_n126), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n130), .b(new_n131), .out0(\s[10] ));
  oai012aa1n06x5               g037(.a(new_n128), .b(new_n127), .c(new_n97), .o1(new_n133));
  nor042aa1n06x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand22aa1n06x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n136), .b(new_n130), .c(new_n133), .out0(\s[11] ));
  nand42aa1n02x5               g042(.a(new_n130), .b(new_n133), .o1(new_n138));
  nand42aa1n02x5               g043(.a(new_n138), .b(new_n136), .o1(new_n139));
  inv040aa1n02x5               g044(.a(new_n134), .o1(new_n140));
  inv000aa1n02x5               g045(.a(new_n136), .o1(new_n141));
  aoai13aa1n03x5               g046(.a(new_n140), .b(new_n141), .c(new_n130), .d(new_n133), .o1(new_n142));
  nor022aa1n06x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand22aa1n06x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  aoib12aa1n02x5               g050(.a(new_n134), .b(new_n144), .c(new_n143), .out0(new_n146));
  aoi022aa1n03x5               g051(.a(new_n142), .b(new_n145), .c(new_n139), .d(new_n146), .o1(\s[12] ));
  nona23aa1n09x5               g052(.a(new_n144), .b(new_n135), .c(new_n134), .d(new_n143), .out0(new_n148));
  nano22aa1n03x7               g053(.a(new_n148), .b(new_n126), .c(new_n129), .out0(new_n149));
  oaoi03aa1n03x5               g054(.a(\a[12] ), .b(\b[11] ), .c(new_n140), .o1(new_n150));
  oabi12aa1n06x5               g055(.a(new_n150), .b(new_n148), .c(new_n133), .out0(new_n151));
  nor042aa1n06x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nand42aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  norb02aa1n02x5               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  aoai13aa1n06x5               g059(.a(new_n154), .b(new_n151), .c(new_n125), .d(new_n149), .o1(new_n155));
  aoi112aa1n02x5               g060(.a(new_n154), .b(new_n151), .c(new_n125), .d(new_n149), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n155), .b(new_n156), .out0(\s[13] ));
  inv000aa1n06x5               g062(.a(new_n152), .o1(new_n158));
  nor002aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nanp02aa1n04x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  norb02aa1n02x5               g065(.a(new_n160), .b(new_n159), .out0(new_n161));
  xnbna2aa1n03x5               g066(.a(new_n161), .b(new_n155), .c(new_n158), .out0(\s[14] ));
  nano23aa1n06x5               g067(.a(new_n152), .b(new_n159), .c(new_n160), .d(new_n153), .out0(new_n163));
  aoai13aa1n06x5               g068(.a(new_n163), .b(new_n151), .c(new_n125), .d(new_n149), .o1(new_n164));
  oaoi03aa1n02x5               g069(.a(\a[14] ), .b(\b[13] ), .c(new_n158), .o1(new_n165));
  inv000aa1n02x5               g070(.a(new_n165), .o1(new_n166));
  xorc02aa1n12x5               g071(.a(\a[15] ), .b(\b[14] ), .out0(new_n167));
  xnbna2aa1n03x5               g072(.a(new_n167), .b(new_n164), .c(new_n166), .out0(\s[15] ));
  nanp02aa1n03x5               g073(.a(new_n164), .b(new_n166), .o1(new_n169));
  nanp02aa1n03x5               g074(.a(new_n169), .b(new_n167), .o1(new_n170));
  inv000aa1d42x5               g075(.a(\a[15] ), .o1(new_n171));
  inv000aa1d42x5               g076(.a(\b[14] ), .o1(new_n172));
  nanp02aa1n02x5               g077(.a(new_n172), .b(new_n171), .o1(new_n173));
  inv000aa1d42x5               g078(.a(new_n167), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n173), .b(new_n174), .c(new_n164), .d(new_n166), .o1(new_n175));
  tech160nm_fixorc02aa1n04x5   g080(.a(\a[16] ), .b(\b[15] ), .out0(new_n176));
  inv000aa1d42x5               g081(.a(\a[16] ), .o1(new_n177));
  inv000aa1d42x5               g082(.a(\b[15] ), .o1(new_n178));
  nanp02aa1n02x5               g083(.a(new_n178), .b(new_n177), .o1(new_n179));
  and002aa1n02x5               g084(.a(\b[15] ), .b(\a[16] ), .o(new_n180));
  aboi22aa1n03x5               g085(.a(new_n180), .b(new_n179), .c(new_n172), .d(new_n171), .out0(new_n181));
  aoi022aa1n03x5               g086(.a(new_n175), .b(new_n176), .c(new_n170), .d(new_n181), .o1(\s[16] ));
  nano23aa1n06x5               g087(.a(new_n134), .b(new_n143), .c(new_n144), .d(new_n135), .out0(new_n183));
  nand23aa1d12x5               g088(.a(new_n163), .b(new_n167), .c(new_n176), .o1(new_n184));
  nano32aa1d15x5               g089(.a(new_n184), .b(new_n183), .c(new_n129), .d(new_n126), .out0(new_n185));
  nand02aa1d08x5               g090(.a(new_n125), .b(new_n185), .o1(new_n186));
  oai022aa1n02x5               g091(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n187));
  nanp02aa1n02x5               g092(.a(\b[14] ), .b(\a[15] ), .o1(new_n188));
  nanp03aa1n03x5               g093(.a(new_n187), .b(new_n160), .c(new_n188), .o1(new_n189));
  aoai13aa1n02x5               g094(.a(new_n179), .b(new_n180), .c(new_n189), .d(new_n173), .o1(new_n190));
  aoib12aa1n12x5               g095(.a(new_n190), .b(new_n151), .c(new_n184), .out0(new_n191));
  xorc02aa1n02x5               g096(.a(\a[17] ), .b(\b[16] ), .out0(new_n192));
  xnbna2aa1n03x5               g097(.a(new_n192), .b(new_n186), .c(new_n191), .out0(\s[17] ));
  inv030aa1d32x5               g098(.a(\a[17] ), .o1(new_n194));
  nanb02aa1n02x5               g099(.a(\b[16] ), .b(new_n194), .out0(new_n195));
  and002aa1n02x5               g100(.a(\b[9] ), .b(\a[10] ), .o(new_n196));
  oab012aa1n02x4               g101(.a(new_n196), .b(new_n97), .c(new_n127), .out0(new_n197));
  tech160nm_fiaoi012aa1n04x5   g102(.a(new_n150), .b(new_n183), .c(new_n197), .o1(new_n198));
  nanp02aa1n02x5               g103(.a(new_n189), .b(new_n173), .o1(new_n199));
  tech160nm_fioaoi03aa1n02p5x5 g104(.a(new_n177), .b(new_n178), .c(new_n199), .o1(new_n200));
  oai012aa1n18x5               g105(.a(new_n200), .b(new_n198), .c(new_n184), .o1(new_n201));
  aoai13aa1n06x5               g106(.a(new_n192), .b(new_n201), .c(new_n125), .d(new_n185), .o1(new_n202));
  xorc02aa1n02x5               g107(.a(\a[18] ), .b(\b[17] ), .out0(new_n203));
  xnbna2aa1n03x5               g108(.a(new_n203), .b(new_n202), .c(new_n195), .out0(\s[18] ));
  inv040aa1d32x5               g109(.a(\a[18] ), .o1(new_n205));
  xroi22aa1d06x4               g110(.a(new_n194), .b(\b[16] ), .c(new_n205), .d(\b[17] ), .out0(new_n206));
  aoai13aa1n06x5               g111(.a(new_n206), .b(new_n201), .c(new_n125), .d(new_n185), .o1(new_n207));
  tech160nm_fioaoi03aa1n03p5x5 g112(.a(\a[18] ), .b(\b[17] ), .c(new_n195), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  xorc02aa1n12x5               g114(.a(\a[19] ), .b(\b[18] ), .out0(new_n210));
  xnbna2aa1n03x5               g115(.a(new_n210), .b(new_n207), .c(new_n209), .out0(\s[19] ));
  xnrc02aa1n02x5               g116(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand02aa1d10x5               g117(.a(new_n186), .b(new_n191), .o1(new_n213));
  aoai13aa1n03x5               g118(.a(new_n210), .b(new_n208), .c(new_n213), .d(new_n206), .o1(new_n214));
  nor002aa1n20x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n210), .o1(new_n217));
  aoai13aa1n02x5               g122(.a(new_n216), .b(new_n217), .c(new_n207), .d(new_n209), .o1(new_n218));
  xorc02aa1n12x5               g123(.a(\a[20] ), .b(\b[19] ), .out0(new_n219));
  inv040aa1d28x5               g124(.a(\a[19] ), .o1(new_n220));
  inv000aa1d42x5               g125(.a(\b[18] ), .o1(new_n221));
  inv040aa1d32x5               g126(.a(\a[20] ), .o1(new_n222));
  inv000aa1d42x5               g127(.a(\b[19] ), .o1(new_n223));
  nand02aa1n04x5               g128(.a(new_n223), .b(new_n222), .o1(new_n224));
  nand42aa1n04x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  aoi022aa1n02x5               g130(.a(new_n224), .b(new_n225), .c(new_n221), .d(new_n220), .o1(new_n226));
  aoi022aa1n03x5               g131(.a(new_n218), .b(new_n219), .c(new_n214), .d(new_n226), .o1(\s[20] ));
  nand23aa1d12x5               g132(.a(new_n206), .b(new_n210), .c(new_n219), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n201), .c(new_n125), .d(new_n185), .o1(new_n230));
  oai112aa1n06x5               g135(.a(new_n224), .b(new_n225), .c(new_n221), .d(new_n220), .o1(new_n231));
  inv000aa1d42x5               g136(.a(\b[17] ), .o1(new_n232));
  oai022aa1d24x5               g137(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n233));
  oai122aa1n12x5               g138(.a(new_n233), .b(\a[19] ), .c(\b[18] ), .d(new_n205), .e(new_n232), .o1(new_n234));
  tech160nm_fioaoi03aa1n03p5x5 g139(.a(new_n222), .b(new_n223), .c(new_n215), .o1(new_n235));
  oai012aa1d24x5               g140(.a(new_n235), .b(new_n234), .c(new_n231), .o1(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  nor002aa1d32x5               g142(.a(\b[20] ), .b(\a[21] ), .o1(new_n238));
  nand22aa1n04x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  norb02aa1d27x5               g144(.a(new_n239), .b(new_n238), .out0(new_n240));
  xnbna2aa1n03x5               g145(.a(new_n240), .b(new_n230), .c(new_n237), .out0(\s[21] ));
  aoai13aa1n06x5               g146(.a(new_n240), .b(new_n236), .c(new_n213), .d(new_n229), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n238), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n240), .o1(new_n244));
  aoai13aa1n02x5               g149(.a(new_n243), .b(new_n244), .c(new_n230), .d(new_n237), .o1(new_n245));
  nor002aa1n04x5               g150(.a(\b[21] ), .b(\a[22] ), .o1(new_n246));
  nand22aa1n06x5               g151(.a(\b[21] ), .b(\a[22] ), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n247), .b(new_n246), .out0(new_n248));
  aoib12aa1n02x5               g153(.a(new_n238), .b(new_n247), .c(new_n246), .out0(new_n249));
  aoi022aa1n03x5               g154(.a(new_n245), .b(new_n248), .c(new_n242), .d(new_n249), .o1(\s[22] ));
  nona23aa1n12x5               g155(.a(new_n247), .b(new_n239), .c(new_n238), .d(new_n246), .out0(new_n251));
  nano32aa1n06x5               g156(.a(new_n251), .b(new_n206), .c(new_n210), .d(new_n219), .out0(new_n252));
  aoai13aa1n06x5               g157(.a(new_n252), .b(new_n201), .c(new_n125), .d(new_n185), .o1(new_n253));
  inv020aa1n02x5               g158(.a(new_n251), .o1(new_n254));
  aoi012aa1n02x5               g159(.a(new_n246), .b(new_n238), .c(new_n247), .o1(new_n255));
  aobi12aa1d24x5               g160(.a(new_n255), .b(new_n236), .c(new_n254), .out0(new_n256));
  nor002aa1n20x5               g161(.a(\b[22] ), .b(\a[23] ), .o1(new_n257));
  nand42aa1n06x5               g162(.a(\b[22] ), .b(\a[23] ), .o1(new_n258));
  norb02aa1d21x5               g163(.a(new_n258), .b(new_n257), .out0(new_n259));
  xnbna2aa1n03x5               g164(.a(new_n259), .b(new_n253), .c(new_n256), .out0(\s[23] ));
  inv000aa1d42x5               g165(.a(new_n256), .o1(new_n261));
  aoai13aa1n06x5               g166(.a(new_n259), .b(new_n261), .c(new_n213), .d(new_n252), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n257), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n259), .o1(new_n264));
  aoai13aa1n02x5               g169(.a(new_n263), .b(new_n264), .c(new_n253), .d(new_n256), .o1(new_n265));
  nor002aa1n04x5               g170(.a(\b[23] ), .b(\a[24] ), .o1(new_n266));
  nand42aa1n06x5               g171(.a(\b[23] ), .b(\a[24] ), .o1(new_n267));
  norb02aa1n02x5               g172(.a(new_n267), .b(new_n266), .out0(new_n268));
  aoib12aa1n02x5               g173(.a(new_n257), .b(new_n267), .c(new_n266), .out0(new_n269));
  aoi022aa1n03x5               g174(.a(new_n265), .b(new_n268), .c(new_n262), .d(new_n269), .o1(\s[24] ));
  nano22aa1n06x5               g175(.a(new_n251), .b(new_n259), .c(new_n268), .out0(new_n271));
  norb02aa1n09x5               g176(.a(new_n271), .b(new_n228), .out0(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n201), .c(new_n125), .d(new_n185), .o1(new_n273));
  aoai13aa1n02x5               g178(.a(new_n258), .b(new_n246), .c(new_n238), .d(new_n247), .o1(new_n274));
  aoi022aa1n03x5               g179(.a(new_n274), .b(new_n263), .c(\a[24] ), .d(\b[23] ), .o1(new_n275));
  aoi112aa1n03x5               g180(.a(new_n266), .b(new_n275), .c(new_n236), .d(new_n271), .o1(new_n276));
  xorc02aa1n12x5               g181(.a(\a[25] ), .b(\b[24] ), .out0(new_n277));
  xnbna2aa1n03x5               g182(.a(new_n277), .b(new_n273), .c(new_n276), .out0(\s[25] ));
  nand02aa1n02x5               g183(.a(new_n236), .b(new_n271), .o1(new_n279));
  nona22aa1n02x4               g184(.a(new_n279), .b(new_n275), .c(new_n266), .out0(new_n280));
  aoai13aa1n03x5               g185(.a(new_n277), .b(new_n280), .c(new_n213), .d(new_n272), .o1(new_n281));
  nor042aa1n03x5               g186(.a(\b[24] ), .b(\a[25] ), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n282), .o1(new_n283));
  inv000aa1d42x5               g188(.a(new_n277), .o1(new_n284));
  aoai13aa1n02x7               g189(.a(new_n283), .b(new_n284), .c(new_n273), .d(new_n276), .o1(new_n285));
  xorc02aa1n02x5               g190(.a(\a[26] ), .b(\b[25] ), .out0(new_n286));
  norp02aa1n02x5               g191(.a(new_n286), .b(new_n282), .o1(new_n287));
  aoi022aa1n03x5               g192(.a(new_n285), .b(new_n286), .c(new_n281), .d(new_n287), .o1(\s[26] ));
  and002aa1n06x5               g193(.a(new_n286), .b(new_n277), .o(new_n289));
  nano22aa1d15x5               g194(.a(new_n228), .b(new_n289), .c(new_n271), .out0(new_n290));
  aoai13aa1n12x5               g195(.a(new_n290), .b(new_n201), .c(new_n125), .d(new_n185), .o1(new_n291));
  oao003aa1n02x5               g196(.a(\a[26] ), .b(\b[25] ), .c(new_n283), .carry(new_n292));
  aobi12aa1n06x5               g197(.a(new_n292), .b(new_n280), .c(new_n289), .out0(new_n293));
  nand42aa1n02x5               g198(.a(new_n293), .b(new_n291), .o1(new_n294));
  xorc02aa1n12x5               g199(.a(\a[27] ), .b(\b[26] ), .out0(new_n295));
  inv000aa1d42x5               g200(.a(new_n295), .o1(new_n296));
  nanp02aa1n02x5               g201(.a(new_n292), .b(new_n296), .o1(new_n297));
  aoi012aa1n02x5               g202(.a(new_n297), .b(new_n280), .c(new_n289), .o1(new_n298));
  aoi022aa1n02x7               g203(.a(new_n294), .b(new_n295), .c(new_n291), .d(new_n298), .o1(\s[27] ));
  oaib12aa1n06x5               g204(.a(new_n292), .b(new_n276), .c(new_n289), .out0(new_n300));
  aoai13aa1n03x5               g205(.a(new_n295), .b(new_n300), .c(new_n213), .d(new_n290), .o1(new_n301));
  norp02aa1n02x5               g206(.a(\b[26] ), .b(\a[27] ), .o1(new_n302));
  inv000aa1n03x5               g207(.a(new_n302), .o1(new_n303));
  aoai13aa1n02x7               g208(.a(new_n303), .b(new_n296), .c(new_n293), .d(new_n291), .o1(new_n304));
  xorc02aa1n02x5               g209(.a(\a[28] ), .b(\b[27] ), .out0(new_n305));
  norp02aa1n02x5               g210(.a(new_n305), .b(new_n302), .o1(new_n306));
  aoi022aa1n03x5               g211(.a(new_n304), .b(new_n305), .c(new_n301), .d(new_n306), .o1(\s[28] ));
  and002aa1n02x5               g212(.a(new_n305), .b(new_n295), .o(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n300), .c(new_n213), .d(new_n290), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n308), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[28] ), .b(\b[27] ), .c(new_n303), .carry(new_n311));
  aoai13aa1n02x7               g216(.a(new_n311), .b(new_n310), .c(new_n293), .d(new_n291), .o1(new_n312));
  xorc02aa1n02x5               g217(.a(\a[29] ), .b(\b[28] ), .out0(new_n313));
  norb02aa1n02x5               g218(.a(new_n311), .b(new_n313), .out0(new_n314));
  aoi022aa1n03x5               g219(.a(new_n312), .b(new_n313), .c(new_n309), .d(new_n314), .o1(\s[29] ));
  xorb03aa1n02x5               g220(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g221(.a(new_n296), .b(new_n305), .c(new_n313), .out0(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n300), .c(new_n213), .d(new_n290), .o1(new_n318));
  inv000aa1d42x5               g223(.a(new_n317), .o1(new_n319));
  oao003aa1n02x5               g224(.a(\a[29] ), .b(\b[28] ), .c(new_n311), .carry(new_n320));
  aoai13aa1n02x7               g225(.a(new_n320), .b(new_n319), .c(new_n293), .d(new_n291), .o1(new_n321));
  xorc02aa1n02x5               g226(.a(\a[30] ), .b(\b[29] ), .out0(new_n322));
  norb02aa1n02x5               g227(.a(new_n320), .b(new_n322), .out0(new_n323));
  aoi022aa1n03x5               g228(.a(new_n321), .b(new_n322), .c(new_n318), .d(new_n323), .o1(\s[30] ));
  xorc02aa1n02x5               g229(.a(\a[31] ), .b(\b[30] ), .out0(new_n325));
  nano32aa1n03x7               g230(.a(new_n296), .b(new_n322), .c(new_n305), .d(new_n313), .out0(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n300), .c(new_n213), .d(new_n290), .o1(new_n327));
  inv000aa1d42x5               g232(.a(new_n326), .o1(new_n328));
  oao003aa1n02x5               g233(.a(\a[30] ), .b(\b[29] ), .c(new_n320), .carry(new_n329));
  aoai13aa1n02x7               g234(.a(new_n329), .b(new_n328), .c(new_n293), .d(new_n291), .o1(new_n330));
  and002aa1n02x5               g235(.a(\b[29] ), .b(\a[30] ), .o(new_n331));
  oabi12aa1n02x5               g236(.a(new_n325), .b(\a[30] ), .c(\b[29] ), .out0(new_n332));
  oab012aa1n02x4               g237(.a(new_n332), .b(new_n320), .c(new_n331), .out0(new_n333));
  aoi022aa1n03x5               g238(.a(new_n330), .b(new_n325), .c(new_n327), .d(new_n333), .o1(\s[31] ));
  xorb03aa1n02x5               g239(.a(new_n100), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  inv000aa1d42x5               g240(.a(new_n107), .o1(new_n336));
  aoi012aa1n02x5               g241(.a(new_n101), .b(new_n100), .c(new_n103), .o1(new_n337));
  aoai13aa1n02x5               g242(.a(new_n336), .b(new_n337), .c(new_n105), .d(new_n104), .o1(\s[4] ));
  norp02aa1n04x5               g243(.a(\b[4] ), .b(\a[5] ), .o1(new_n339));
  inv000aa1d42x5               g244(.a(new_n339), .o1(new_n340));
  aboi22aa1n03x5               g245(.a(new_n107), .b(new_n104), .c(new_n340), .d(new_n108), .out0(new_n341));
  nona23aa1n09x5               g246(.a(new_n104), .b(new_n108), .c(new_n107), .d(new_n339), .out0(new_n342));
  norb02aa1n02x5               g247(.a(new_n342), .b(new_n341), .out0(\s[5] ));
  xorc02aa1n02x5               g248(.a(\a[6] ), .b(\b[5] ), .out0(new_n344));
  nanb02aa1n06x5               g249(.a(new_n121), .b(new_n342), .out0(new_n345));
  aoai13aa1n02x5               g250(.a(new_n345), .b(new_n344), .c(new_n340), .d(new_n342), .o1(\s[6] ));
  norb02aa1n02x5               g251(.a(new_n109), .b(new_n117), .out0(new_n347));
  xobna2aa1n03x5               g252(.a(new_n347), .b(new_n345), .c(new_n120), .out0(\s[7] ));
  aoai13aa1n02x5               g253(.a(new_n123), .b(new_n117), .c(new_n345), .d(new_n122), .o1(new_n349));
  aoi112aa1n02x5               g254(.a(new_n123), .b(new_n117), .c(new_n345), .d(new_n122), .o1(new_n350));
  norb02aa1n02x5               g255(.a(new_n349), .b(new_n350), .out0(\s[8] ));
  xorb03aa1n02x5               g256(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


