// Benchmark "adder" written by ABC on Thu Jul 18 09:53:33 2024

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
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n267, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n291, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n351,
    new_n352, new_n354, new_n356, new_n358, new_n359, new_n360, new_n361,
    new_n363, new_n365, new_n366, new_n367;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand42aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nand02aa1d08x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  tech160nm_fiaoi012aa1n05x5   g003(.a(new_n98), .b(\a[2] ), .c(\b[1] ), .o1(new_n99));
  oai012aa1n06x5               g004(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .o1(new_n100));
  nand42aa1n03x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nand42aa1n06x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nor042aa1n12x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nano22aa1n03x7               g008(.a(new_n103), .b(new_n101), .c(new_n102), .out0(new_n104));
  oai022aa1n02x5               g009(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n105));
  aoi012aa1n12x5               g010(.a(new_n105), .b(new_n100), .c(new_n104), .o1(new_n106));
  nand42aa1n03x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nor002aa1n06x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nand42aa1n06x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nanb03aa1n06x5               g014(.a(new_n108), .b(new_n109), .c(new_n107), .out0(new_n110));
  tech160nm_fixnrc02aa1n02p5x5 g015(.a(\b[7] ), .b(\a[8] ), .out0(new_n111));
  nor042aa1n06x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nand02aa1d04x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nanp02aa1n04x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nor022aa1n12x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nano23aa1n06x5               g020(.a(new_n115), .b(new_n112), .c(new_n113), .d(new_n114), .out0(new_n116));
  nona22aa1n09x5               g021(.a(new_n116), .b(new_n110), .c(new_n111), .out0(new_n117));
  orn002aa1n02x5               g022(.a(\a[8] ), .b(\b[7] ), .o(new_n118));
  inv000aa1d42x5               g023(.a(new_n115), .o1(new_n119));
  oai012aa1n04x7               g024(.a(new_n109), .b(new_n112), .c(new_n108), .o1(new_n120));
  aob012aa1n02x5               g025(.a(new_n114), .b(\b[7] ), .c(\a[8] ), .out0(new_n121));
  aoi012aa1n02x5               g026(.a(new_n121), .b(new_n120), .c(new_n119), .o1(new_n122));
  nor042aa1n02x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  norb02aa1n02x5               g028(.a(new_n97), .b(new_n123), .out0(new_n124));
  nanb03aa1n02x5               g029(.a(new_n122), .b(new_n124), .c(new_n118), .out0(new_n125));
  oabi12aa1n06x5               g030(.a(new_n125), .b(new_n106), .c(new_n117), .out0(new_n126));
  nor042aa1n02x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand42aa1n03x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nanb02aa1n02x5               g033(.a(new_n127), .b(new_n128), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n126), .c(new_n97), .out0(\s[10] ));
  nor002aa1d24x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand42aa1n02x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nanb02aa1n02x5               g037(.a(new_n131), .b(new_n132), .out0(new_n133));
  aoai13aa1n02x5               g038(.a(new_n128), .b(new_n127), .c(new_n126), .d(new_n97), .o1(new_n134));
  nano22aa1n02x4               g039(.a(new_n131), .b(new_n128), .c(new_n132), .out0(new_n135));
  aoai13aa1n02x5               g040(.a(new_n135), .b(new_n129), .c(new_n126), .d(new_n97), .o1(new_n136));
  aobi12aa1n02x5               g041(.a(new_n136), .b(new_n134), .c(new_n133), .out0(\s[11] ));
  inv000aa1d42x5               g042(.a(new_n131), .o1(new_n138));
  nanp02aa1n03x5               g043(.a(new_n136), .b(new_n138), .o1(new_n139));
  nor002aa1n12x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  and002aa1n12x5               g045(.a(\b[11] ), .b(\a[12] ), .o(new_n141));
  nor042aa1n02x5               g046(.a(new_n141), .b(new_n140), .o1(new_n142));
  oab012aa1n02x4               g047(.a(new_n131), .b(new_n141), .c(new_n140), .out0(new_n143));
  aoi022aa1n02x5               g048(.a(new_n139), .b(new_n142), .c(new_n136), .d(new_n143), .o1(\s[12] ));
  aoai13aa1n06x5               g049(.a(new_n118), .b(new_n121), .c(new_n120), .d(new_n119), .o1(new_n145));
  oabi12aa1n18x5               g050(.a(new_n145), .b(new_n117), .c(new_n106), .out0(new_n146));
  nano23aa1n06x5               g051(.a(new_n133), .b(new_n129), .c(new_n142), .d(new_n124), .out0(new_n147));
  tech160nm_fiaoi012aa1n04x5   g052(.a(new_n131), .b(\a[10] ), .c(\b[9] ), .o1(new_n148));
  aoi112aa1n06x5               g053(.a(new_n141), .b(new_n140), .c(\a[11] ), .d(\b[10] ), .o1(new_n149));
  oai112aa1n06x5               g054(.a(new_n149), .b(new_n148), .c(new_n127), .d(new_n123), .o1(new_n150));
  oab012aa1n06x5               g055(.a(new_n140), .b(new_n138), .c(new_n141), .out0(new_n151));
  nanp02aa1n02x5               g056(.a(new_n150), .b(new_n151), .o1(new_n152));
  xorc02aa1n12x5               g057(.a(\a[13] ), .b(\b[12] ), .out0(new_n153));
  aoai13aa1n06x5               g058(.a(new_n153), .b(new_n152), .c(new_n146), .d(new_n147), .o1(new_n154));
  nano22aa1n02x4               g059(.a(new_n153), .b(new_n150), .c(new_n151), .out0(new_n155));
  aobi12aa1n02x5               g060(.a(new_n155), .b(new_n146), .c(new_n147), .out0(new_n156));
  norb02aa1n02x5               g061(.a(new_n154), .b(new_n156), .out0(\s[13] ));
  inv000aa1d42x5               g062(.a(\a[13] ), .o1(new_n158));
  inv000aa1d42x5               g063(.a(\b[12] ), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(new_n159), .b(new_n158), .o1(new_n160));
  xorc02aa1n12x5               g065(.a(\a[14] ), .b(\b[13] ), .out0(new_n161));
  xnbna2aa1n03x5               g066(.a(new_n161), .b(new_n154), .c(new_n160), .out0(\s[14] ));
  oai022aa1d24x5               g067(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  nanp02aa1n09x5               g069(.a(new_n154), .b(new_n164), .o1(new_n165));
  nanp02aa1n02x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nanp02aa1n24x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nor042aa1n06x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nano22aa1n02x4               g073(.a(new_n168), .b(new_n166), .c(new_n167), .out0(new_n169));
  nand22aa1n03x5               g074(.a(new_n165), .b(new_n169), .o1(new_n170));
  inv000aa1n03x5               g075(.a(new_n168), .o1(new_n171));
  aoi022aa1n02x5               g076(.a(new_n165), .b(new_n166), .c(new_n171), .d(new_n167), .o1(new_n172));
  norb02aa1n03x4               g077(.a(new_n170), .b(new_n172), .out0(\s[15] ));
  inv000aa1n02x5               g078(.a(new_n169), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n171), .b(new_n174), .c(new_n154), .d(new_n164), .o1(new_n175));
  nor042aa1n04x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nanp02aa1n12x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  norp02aa1n02x5               g083(.a(new_n178), .b(new_n168), .o1(new_n179));
  aoi022aa1n03x5               g084(.a(new_n175), .b(new_n178), .c(new_n170), .d(new_n179), .o1(\s[16] ));
  tech160nm_finand02aa1n03p5x5 g085(.a(new_n100), .b(new_n104), .o1(new_n181));
  nanb02aa1n06x5               g086(.a(new_n105), .b(new_n181), .out0(new_n182));
  nona23aa1n02x4               g087(.a(new_n114), .b(new_n113), .c(new_n115), .d(new_n112), .out0(new_n183));
  nor043aa1n02x5               g088(.a(new_n183), .b(new_n111), .c(new_n110), .o1(new_n184));
  norb02aa1n03x5               g089(.a(new_n132), .b(new_n131), .out0(new_n185));
  nano23aa1n03x7               g090(.a(new_n127), .b(new_n123), .c(new_n128), .d(new_n97), .out0(new_n186));
  nano23aa1n06x5               g091(.a(new_n176), .b(new_n168), .c(new_n177), .d(new_n167), .out0(new_n187));
  nand23aa1n06x5               g092(.a(new_n187), .b(new_n153), .c(new_n161), .o1(new_n188));
  nano32aa1d12x5               g093(.a(new_n188), .b(new_n186), .c(new_n142), .d(new_n185), .out0(new_n189));
  aoai13aa1n06x5               g094(.a(new_n189), .b(new_n145), .c(new_n182), .d(new_n184), .o1(new_n190));
  nanb03aa1n02x5               g095(.a(new_n176), .b(new_n177), .c(new_n167), .out0(new_n191));
  nano32aa1n03x7               g096(.a(new_n191), .b(new_n163), .c(new_n171), .d(new_n166), .out0(new_n192));
  aoi012aa1n02x5               g097(.a(new_n176), .b(new_n168), .c(new_n177), .o1(new_n193));
  norb02aa1n06x5               g098(.a(new_n193), .b(new_n192), .out0(new_n194));
  aoai13aa1n12x5               g099(.a(new_n194), .b(new_n188), .c(new_n150), .d(new_n151), .o1(new_n195));
  inv000aa1n09x5               g100(.a(new_n195), .o1(new_n196));
  nanp02aa1n06x5               g101(.a(new_n190), .b(new_n196), .o1(new_n197));
  xorc02aa1n12x5               g102(.a(\a[17] ), .b(\b[16] ), .out0(new_n198));
  nona22aa1n02x4               g103(.a(new_n193), .b(new_n192), .c(new_n198), .out0(new_n199));
  aoib12aa1n02x5               g104(.a(new_n199), .b(new_n152), .c(new_n188), .out0(new_n200));
  aoi022aa1n02x5               g105(.a(new_n197), .b(new_n198), .c(new_n190), .d(new_n200), .o1(\s[17] ));
  nor042aa1d18x5               g106(.a(\b[16] ), .b(\a[17] ), .o1(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  aoai13aa1n06x5               g108(.a(new_n198), .b(new_n195), .c(new_n146), .d(new_n189), .o1(new_n204));
  nor042aa1n04x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  nanp02aa1n12x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  norb02aa1n03x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  xnbna2aa1n03x5               g112(.a(new_n207), .b(new_n204), .c(new_n203), .out0(\s[18] ));
  nor042aa1n09x5               g113(.a(new_n205), .b(new_n202), .o1(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  nand02aa1n06x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  oai012aa1d24x5               g116(.a(new_n206), .b(\b[18] ), .c(\a[19] ), .o1(new_n212));
  norb02aa1n06x5               g117(.a(new_n211), .b(new_n212), .out0(new_n213));
  aoai13aa1n06x5               g118(.a(new_n213), .b(new_n210), .c(new_n197), .d(new_n198), .o1(new_n214));
  inv000aa1d42x5               g119(.a(\a[19] ), .o1(new_n215));
  inv000aa1d42x5               g120(.a(\b[18] ), .o1(new_n216));
  nand02aa1d12x5               g121(.a(new_n216), .b(new_n215), .o1(new_n217));
  nanp02aa1n02x5               g122(.a(new_n217), .b(new_n211), .o1(new_n218));
  aoai13aa1n02x5               g123(.a(new_n206), .b(new_n210), .c(new_n197), .d(new_n198), .o1(new_n219));
  aobi12aa1n02x7               g124(.a(new_n214), .b(new_n219), .c(new_n218), .out0(\s[19] ));
  xnrc02aa1n02x5               g125(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g126(.a(new_n213), .o1(new_n222));
  aoai13aa1n02x5               g127(.a(new_n217), .b(new_n222), .c(new_n204), .d(new_n209), .o1(new_n223));
  tech160nm_finor002aa1n05x5   g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  nanp02aa1n06x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  norb02aa1n03x5               g130(.a(new_n225), .b(new_n224), .out0(new_n226));
  inv000aa1d42x5               g131(.a(\a[20] ), .o1(new_n227));
  inv000aa1d42x5               g132(.a(\b[19] ), .o1(new_n228));
  nanp02aa1n02x5               g133(.a(new_n228), .b(new_n227), .o1(new_n229));
  aoi022aa1n02x5               g134(.a(new_n229), .b(new_n225), .c(new_n216), .d(new_n215), .o1(new_n230));
  aoi022aa1n02x5               g135(.a(new_n223), .b(new_n226), .c(new_n214), .d(new_n230), .o1(\s[20] ));
  nano32aa1n03x7               g136(.a(new_n218), .b(new_n198), .c(new_n226), .d(new_n207), .out0(new_n232));
  aoai13aa1n02x5               g137(.a(new_n232), .b(new_n195), .c(new_n146), .d(new_n189), .o1(new_n233));
  nanb03aa1n12x5               g138(.a(new_n224), .b(new_n225), .c(new_n211), .out0(new_n234));
  oaoi03aa1n12x5               g139(.a(\a[20] ), .b(\b[19] ), .c(new_n217), .o1(new_n235));
  inv040aa1n02x5               g140(.a(new_n235), .o1(new_n236));
  oai013aa1d12x5               g141(.a(new_n236), .b(new_n234), .c(new_n209), .d(new_n212), .o1(new_n237));
  norp02aa1n24x5               g142(.a(\b[20] ), .b(\a[21] ), .o1(new_n238));
  nand42aa1n02x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  norb02aa1d27x5               g144(.a(new_n239), .b(new_n238), .out0(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n237), .c(new_n197), .d(new_n232), .o1(new_n241));
  nano22aa1n02x4               g146(.a(new_n224), .b(new_n211), .c(new_n225), .out0(new_n242));
  oab012aa1n02x4               g147(.a(new_n212), .b(new_n202), .c(new_n205), .out0(new_n243));
  aoi112aa1n02x5               g148(.a(new_n235), .b(new_n240), .c(new_n243), .d(new_n242), .o1(new_n244));
  aobi12aa1n02x7               g149(.a(new_n241), .b(new_n244), .c(new_n233), .out0(\s[21] ));
  inv000aa1d42x5               g150(.a(new_n237), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n238), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n240), .o1(new_n248));
  aoai13aa1n02x5               g153(.a(new_n247), .b(new_n248), .c(new_n233), .d(new_n246), .o1(new_n249));
  nor042aa1n03x5               g154(.a(\b[21] ), .b(\a[22] ), .o1(new_n250));
  nanp02aa1n03x5               g155(.a(\b[21] ), .b(\a[22] ), .o1(new_n251));
  norb02aa1n02x5               g156(.a(new_n251), .b(new_n250), .out0(new_n252));
  aoib12aa1n02x5               g157(.a(new_n238), .b(new_n251), .c(new_n250), .out0(new_n253));
  aoi022aa1n02x5               g158(.a(new_n249), .b(new_n252), .c(new_n241), .d(new_n253), .o1(\s[22] ));
  inv000aa1n02x5               g159(.a(new_n232), .o1(new_n255));
  nona23aa1n03x5               g160(.a(new_n251), .b(new_n239), .c(new_n238), .d(new_n250), .out0(new_n256));
  nor042aa1n03x5               g161(.a(new_n255), .b(new_n256), .o1(new_n257));
  aoai13aa1n06x5               g162(.a(new_n257), .b(new_n195), .c(new_n146), .d(new_n189), .o1(new_n258));
  nano23aa1n06x5               g163(.a(new_n238), .b(new_n250), .c(new_n251), .d(new_n239), .out0(new_n259));
  oaih12aa1n06x5               g164(.a(new_n251), .b(new_n250), .c(new_n238), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  aoi012aa1n02x5               g166(.a(new_n261), .b(new_n237), .c(new_n259), .o1(new_n262));
  xorc02aa1n02x5               g167(.a(\a[23] ), .b(\b[22] ), .out0(new_n263));
  aob012aa1n03x5               g168(.a(new_n263), .b(new_n258), .c(new_n262), .out0(new_n264));
  aoi112aa1n02x5               g169(.a(new_n263), .b(new_n261), .c(new_n237), .d(new_n259), .o1(new_n265));
  aobi12aa1n02x5               g170(.a(new_n264), .b(new_n265), .c(new_n258), .out0(\s[23] ));
  inv000aa1d42x5               g171(.a(\a[23] ), .o1(new_n267));
  inv000aa1d42x5               g172(.a(\b[22] ), .o1(new_n268));
  nanp02aa1n02x5               g173(.a(new_n268), .b(new_n267), .o1(new_n269));
  and002aa1n02x5               g174(.a(\b[22] ), .b(\a[23] ), .o(new_n270));
  aoai13aa1n02x5               g175(.a(new_n269), .b(new_n270), .c(new_n258), .d(new_n262), .o1(new_n271));
  xorc02aa1n02x5               g176(.a(\a[24] ), .b(\b[23] ), .out0(new_n272));
  norb02aa1n02x5               g177(.a(new_n269), .b(new_n272), .out0(new_n273));
  aoi022aa1n02x5               g178(.a(new_n271), .b(new_n272), .c(new_n264), .d(new_n273), .o1(\s[24] ));
  nano22aa1n03x7               g179(.a(new_n256), .b(new_n263), .c(new_n272), .out0(new_n275));
  norb02aa1n03x5               g180(.a(new_n275), .b(new_n255), .out0(new_n276));
  aoai13aa1n04x5               g181(.a(new_n276), .b(new_n195), .c(new_n146), .d(new_n189), .o1(new_n277));
  nanp02aa1n02x5               g182(.a(\b[23] ), .b(\a[24] ), .o1(new_n278));
  aoai13aa1n06x5               g183(.a(new_n259), .b(new_n235), .c(new_n243), .d(new_n242), .o1(new_n279));
  oa0022aa1n02x5               g184(.a(\a[24] ), .b(\b[23] ), .c(\a[23] ), .d(\b[22] ), .o(new_n280));
  aoai13aa1n04x5               g185(.a(new_n280), .b(new_n270), .c(new_n279), .d(new_n260), .o1(new_n281));
  nand22aa1n02x5               g186(.a(new_n281), .b(new_n278), .o1(new_n282));
  xorc02aa1n12x5               g187(.a(\a[25] ), .b(\b[24] ), .out0(new_n283));
  xnbna2aa1n03x5               g188(.a(new_n283), .b(new_n277), .c(new_n282), .out0(\s[25] ));
  aob012aa1n03x5               g189(.a(new_n283), .b(new_n277), .c(new_n282), .out0(new_n285));
  nor042aa1n06x5               g190(.a(\b[24] ), .b(\a[25] ), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n286), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n283), .o1(new_n288));
  aoai13aa1n02x5               g193(.a(new_n287), .b(new_n288), .c(new_n277), .d(new_n282), .o1(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[25] ), .b(\a[26] ), .out0(new_n290));
  norb02aa1n02x5               g195(.a(new_n290), .b(new_n286), .out0(new_n291));
  aboi22aa1n03x5               g196(.a(new_n290), .b(new_n289), .c(new_n285), .d(new_n291), .out0(\s[26] ));
  norb02aa1n02x5               g197(.a(new_n283), .b(new_n290), .out0(new_n293));
  nano22aa1n03x7               g198(.a(new_n255), .b(new_n275), .c(new_n293), .out0(new_n294));
  aoai13aa1n06x5               g199(.a(new_n294), .b(new_n195), .c(new_n146), .d(new_n189), .o1(new_n295));
  nanp02aa1n02x5               g200(.a(\b[25] ), .b(\a[26] ), .o1(new_n296));
  nanp02aa1n02x5               g201(.a(\b[24] ), .b(\a[25] ), .o1(new_n297));
  oai012aa1n02x5               g202(.a(new_n297), .b(\b[25] ), .c(\a[26] ), .o1(new_n298));
  nano23aa1d12x5               g203(.a(new_n298), .b(new_n286), .c(new_n278), .d(new_n296), .out0(new_n299));
  oao003aa1n02x5               g204(.a(\a[26] ), .b(\b[25] ), .c(new_n287), .carry(new_n300));
  inv000aa1d42x5               g205(.a(new_n300), .o1(new_n301));
  tech160nm_fiaoi012aa1n05x5   g206(.a(new_n301), .b(new_n281), .c(new_n299), .o1(new_n302));
  nanp02aa1n02x5               g207(.a(new_n302), .b(new_n295), .o1(new_n303));
  xorc02aa1n12x5               g208(.a(\a[27] ), .b(\b[26] ), .out0(new_n304));
  aoi112aa1n02x5               g209(.a(new_n304), .b(new_n301), .c(new_n281), .d(new_n299), .o1(new_n305));
  aoi022aa1n02x5               g210(.a(new_n303), .b(new_n304), .c(new_n295), .d(new_n305), .o1(\s[27] ));
  aobi12aa1n06x5               g211(.a(new_n294), .b(new_n190), .c(new_n196), .out0(new_n307));
  inv000aa1d42x5               g212(.a(new_n270), .o1(new_n308));
  aoai13aa1n06x5               g213(.a(new_n308), .b(new_n261), .c(new_n237), .d(new_n259), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n299), .o1(new_n310));
  aoai13aa1n06x5               g215(.a(new_n300), .b(new_n310), .c(new_n309), .d(new_n280), .o1(new_n311));
  oai012aa1n03x5               g216(.a(new_n304), .b(new_n311), .c(new_n307), .o1(new_n312));
  nor042aa1n06x5               g217(.a(\b[26] ), .b(\a[27] ), .o1(new_n313));
  inv000aa1n06x5               g218(.a(new_n313), .o1(new_n314));
  inv000aa1d42x5               g219(.a(new_n304), .o1(new_n315));
  aoai13aa1n03x5               g220(.a(new_n314), .b(new_n315), .c(new_n302), .d(new_n295), .o1(new_n316));
  xorc02aa1n02x5               g221(.a(\a[28] ), .b(\b[27] ), .out0(new_n317));
  norp02aa1n02x5               g222(.a(new_n317), .b(new_n313), .o1(new_n318));
  aoi022aa1n03x5               g223(.a(new_n316), .b(new_n317), .c(new_n312), .d(new_n318), .o1(\s[28] ));
  and002aa1n02x5               g224(.a(new_n317), .b(new_n304), .o(new_n320));
  tech160nm_fioai012aa1n04x5   g225(.a(new_n320), .b(new_n311), .c(new_n307), .o1(new_n321));
  inv000aa1d42x5               g226(.a(new_n320), .o1(new_n322));
  oao003aa1n03x5               g227(.a(\a[28] ), .b(\b[27] ), .c(new_n314), .carry(new_n323));
  aoai13aa1n03x5               g228(.a(new_n323), .b(new_n322), .c(new_n302), .d(new_n295), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[29] ), .b(\b[28] ), .out0(new_n325));
  norb02aa1n02x5               g230(.a(new_n323), .b(new_n325), .out0(new_n326));
  aoi022aa1n03x5               g231(.a(new_n324), .b(new_n325), .c(new_n321), .d(new_n326), .o1(\s[29] ));
  xorb03aa1n02x5               g232(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g233(.a(new_n315), .b(new_n317), .c(new_n325), .out0(new_n329));
  oai012aa1n03x5               g234(.a(new_n329), .b(new_n311), .c(new_n307), .o1(new_n330));
  inv000aa1n02x5               g235(.a(new_n329), .o1(new_n331));
  tech160nm_fioaoi03aa1n03p5x5 g236(.a(\a[29] ), .b(\b[28] ), .c(new_n323), .o1(new_n332));
  inv000aa1d42x5               g237(.a(new_n332), .o1(new_n333));
  aoai13aa1n03x5               g238(.a(new_n333), .b(new_n331), .c(new_n302), .d(new_n295), .o1(new_n334));
  xorc02aa1n02x5               g239(.a(\a[30] ), .b(\b[29] ), .out0(new_n335));
  and002aa1n02x5               g240(.a(\b[28] ), .b(\a[29] ), .o(new_n336));
  oabi12aa1n02x5               g241(.a(new_n335), .b(\a[29] ), .c(\b[28] ), .out0(new_n337));
  oab012aa1n02x4               g242(.a(new_n337), .b(new_n323), .c(new_n336), .out0(new_n338));
  aoi022aa1n03x5               g243(.a(new_n334), .b(new_n335), .c(new_n330), .d(new_n338), .o1(\s[30] ));
  nano32aa1n02x4               g244(.a(new_n315), .b(new_n335), .c(new_n317), .d(new_n325), .out0(new_n340));
  oai012aa1n03x5               g245(.a(new_n340), .b(new_n311), .c(new_n307), .o1(new_n341));
  xorc02aa1n02x5               g246(.a(\a[31] ), .b(\b[30] ), .out0(new_n342));
  inv000aa1d42x5               g247(.a(\a[30] ), .o1(new_n343));
  inv000aa1d42x5               g248(.a(\b[29] ), .o1(new_n344));
  oabi12aa1n02x5               g249(.a(new_n342), .b(\a[30] ), .c(\b[29] ), .out0(new_n345));
  oaoi13aa1n02x5               g250(.a(new_n345), .b(new_n332), .c(new_n343), .d(new_n344), .o1(new_n346));
  inv000aa1n02x5               g251(.a(new_n340), .o1(new_n347));
  oaoi03aa1n02x5               g252(.a(new_n343), .b(new_n344), .c(new_n332), .o1(new_n348));
  aoai13aa1n06x5               g253(.a(new_n348), .b(new_n347), .c(new_n302), .d(new_n295), .o1(new_n349));
  aoi022aa1n03x5               g254(.a(new_n349), .b(new_n342), .c(new_n341), .d(new_n346), .o1(\s[31] ));
  inv000aa1d42x5               g255(.a(new_n103), .o1(new_n351));
  aoi022aa1n02x5               g256(.a(new_n100), .b(new_n101), .c(new_n351), .d(new_n102), .o1(new_n352));
  norb02aa1n02x5               g257(.a(new_n181), .b(new_n352), .out0(\s[3] ));
  xorc02aa1n02x5               g258(.a(\a[4] ), .b(\b[3] ), .out0(new_n354));
  xnbna2aa1n03x5               g259(.a(new_n354), .b(new_n181), .c(new_n351), .out0(\s[4] ));
  norb02aa1n02x5               g260(.a(new_n113), .b(new_n112), .out0(new_n356));
  xobna2aa1n03x5               g261(.a(new_n356), .b(new_n182), .c(new_n107), .out0(\s[5] ));
  norb02aa1n02x5               g262(.a(new_n109), .b(new_n108), .out0(new_n358));
  aoi013aa1n02x4               g263(.a(new_n112), .b(new_n182), .c(new_n107), .d(new_n356), .o1(new_n359));
  nanp03aa1n02x5               g264(.a(new_n182), .b(new_n107), .c(new_n356), .o1(new_n360));
  nona23aa1n06x5               g265(.a(new_n360), .b(new_n109), .c(new_n108), .d(new_n112), .out0(new_n361));
  oai012aa1n02x5               g266(.a(new_n361), .b(new_n359), .c(new_n358), .o1(\s[6] ));
  norb02aa1n02x5               g267(.a(new_n114), .b(new_n115), .out0(new_n363));
  xobna2aa1n03x5               g268(.a(new_n363), .b(new_n361), .c(new_n109), .out0(\s[7] ));
  nanp03aa1n02x5               g269(.a(new_n361), .b(new_n109), .c(new_n363), .o1(new_n365));
  aoi012aa1n02x5               g270(.a(new_n111), .b(new_n365), .c(new_n119), .o1(new_n366));
  norb02aa1n02x5               g271(.a(new_n111), .b(new_n115), .out0(new_n367));
  aoi012aa1n02x5               g272(.a(new_n366), .b(new_n365), .c(new_n367), .o1(\s[8] ));
  oaib12aa1n02x5               g273(.a(new_n126), .b(new_n124), .c(new_n146), .out0(\s[9] ));
endmodule


