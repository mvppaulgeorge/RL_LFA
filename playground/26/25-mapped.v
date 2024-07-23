// Benchmark "adder" written by ABC on Thu Jul 18 01:29:59 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n317, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n342, new_n343, new_n344,
    new_n345, new_n347, new_n348, new_n350, new_n352, new_n353, new_n354,
    new_n355, new_n357, new_n359, new_n360, new_n361, new_n363, new_n364;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  nanb02aa1n02x5               g002(.a(\b[8] ), .b(new_n97), .out0(new_n98));
  and002aa1n12x5               g003(.a(\b[0] ), .b(\a[1] ), .o(new_n99));
  oaoi03aa1n12x5               g004(.a(\a[2] ), .b(\b[1] ), .c(new_n99), .o1(new_n100));
  nor042aa1n03x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nand02aa1n08x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  norb02aa1n03x5               g007(.a(new_n102), .b(new_n101), .out0(new_n103));
  nor042aa1n04x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n04x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  norb02aa1n03x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nanp03aa1d12x5               g011(.a(new_n100), .b(new_n103), .c(new_n106), .o1(new_n107));
  aoi012aa1n12x5               g012(.a(new_n101), .b(new_n104), .c(new_n102), .o1(new_n108));
  nor002aa1n16x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nand42aa1n20x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nor002aa1n03x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nanp02aa1n12x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nano23aa1n06x5               g017(.a(new_n109), .b(new_n111), .c(new_n112), .d(new_n110), .out0(new_n113));
  xorc02aa1n12x5               g018(.a(\a[8] ), .b(\b[7] ), .out0(new_n114));
  nor042aa1d18x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nand42aa1n16x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  norb02aa1n03x5               g021(.a(new_n116), .b(new_n115), .out0(new_n117));
  nand23aa1n03x5               g022(.a(new_n113), .b(new_n114), .c(new_n117), .o1(new_n118));
  nano22aa1n02x5               g023(.a(new_n115), .b(new_n110), .c(new_n116), .out0(new_n119));
  oai022aa1d18x5               g024(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\a[8] ), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\b[7] ), .o1(new_n122));
  oaoi03aa1n12x5               g027(.a(new_n121), .b(new_n122), .c(new_n115), .o1(new_n123));
  inv000aa1n02x5               g028(.a(new_n123), .o1(new_n124));
  aoi013aa1n06x4               g029(.a(new_n124), .b(new_n119), .c(new_n114), .d(new_n120), .o1(new_n125));
  aoai13aa1n12x5               g030(.a(new_n125), .b(new_n118), .c(new_n107), .d(new_n108), .o1(new_n126));
  xnrc02aa1n12x5               g031(.a(\b[8] ), .b(\a[9] ), .out0(new_n127));
  inv000aa1d42x5               g032(.a(new_n127), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(new_n126), .b(new_n128), .o1(new_n129));
  tech160nm_fixorc02aa1n03p5x5 g034(.a(\a[10] ), .b(\b[9] ), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n129), .c(new_n98), .out0(\s[10] ));
  oao003aa1n02x5               g036(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .carry(new_n132));
  nanp03aa1n03x5               g037(.a(new_n126), .b(new_n128), .c(new_n130), .o1(new_n133));
  nor042aa1n02x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand22aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n136), .b(new_n133), .c(new_n132), .out0(\s[11] ));
  aob012aa1n03x5               g042(.a(new_n136), .b(new_n133), .c(new_n132), .out0(new_n138));
  nor042aa1n03x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nand02aa1n06x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  aoib12aa1n02x5               g046(.a(new_n134), .b(new_n140), .c(new_n139), .out0(new_n142));
  tech160nm_fioai012aa1n03p5x5 g047(.a(new_n138), .b(\b[10] ), .c(\a[11] ), .o1(new_n143));
  aoi022aa1n03x5               g048(.a(new_n143), .b(new_n141), .c(new_n138), .d(new_n142), .o1(\s[12] ));
  nano32aa1n03x7               g049(.a(new_n127), .b(new_n141), .c(new_n130), .d(new_n136), .out0(new_n145));
  oai022aa1n02x5               g050(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n146));
  nano22aa1n03x5               g051(.a(new_n139), .b(new_n135), .c(new_n140), .out0(new_n147));
  aoi012aa1n02x5               g052(.a(new_n134), .b(\a[10] ), .c(\b[9] ), .o1(new_n148));
  nand23aa1n03x5               g053(.a(new_n147), .b(new_n146), .c(new_n148), .o1(new_n149));
  tech160nm_fiaoi012aa1n04x5   g054(.a(new_n139), .b(new_n134), .c(new_n140), .o1(new_n150));
  nanp02aa1n02x5               g055(.a(new_n149), .b(new_n150), .o1(new_n151));
  nor042aa1n04x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  tech160nm_finand02aa1n05x5   g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanb02aa1n02x5               g058(.a(new_n152), .b(new_n153), .out0(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  aoai13aa1n06x5               g060(.a(new_n155), .b(new_n151), .c(new_n126), .d(new_n145), .o1(new_n156));
  nanp03aa1n02x5               g061(.a(new_n149), .b(new_n150), .c(new_n154), .o1(new_n157));
  aoi012aa1n02x5               g062(.a(new_n157), .b(new_n126), .c(new_n145), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n156), .b(new_n158), .out0(\s[13] ));
  orn002aa1n02x5               g064(.a(\a[13] ), .b(\b[12] ), .o(new_n160));
  nor042aa1n04x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand42aa1n08x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  xnbna2aa1n03x5               g068(.a(new_n163), .b(new_n156), .c(new_n160), .out0(\s[14] ));
  nano23aa1n06x5               g069(.a(new_n152), .b(new_n161), .c(new_n162), .d(new_n153), .out0(new_n165));
  aoai13aa1n06x5               g070(.a(new_n165), .b(new_n151), .c(new_n126), .d(new_n145), .o1(new_n166));
  oai012aa1n02x5               g071(.a(new_n162), .b(new_n161), .c(new_n152), .o1(new_n167));
  nor002aa1n20x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nand42aa1n10x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  norb02aa1n15x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  xnbna2aa1n03x5               g075(.a(new_n170), .b(new_n166), .c(new_n167), .out0(\s[15] ));
  aob012aa1n03x5               g076(.a(new_n170), .b(new_n166), .c(new_n167), .out0(new_n172));
  tech160nm_fixorc02aa1n04x5   g077(.a(\a[16] ), .b(\b[15] ), .out0(new_n173));
  inv000aa1d42x5               g078(.a(\a[16] ), .o1(new_n174));
  inv000aa1d42x5               g079(.a(\b[15] ), .o1(new_n175));
  nanp02aa1n02x5               g080(.a(new_n175), .b(new_n174), .o1(new_n176));
  nanp02aa1n02x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  aoi012aa1n02x5               g082(.a(new_n168), .b(new_n176), .c(new_n177), .o1(new_n178));
  inv000aa1n02x5               g083(.a(new_n168), .o1(new_n179));
  inv000aa1d42x5               g084(.a(new_n170), .o1(new_n180));
  aoai13aa1n02x5               g085(.a(new_n179), .b(new_n180), .c(new_n166), .d(new_n167), .o1(new_n181));
  aoi022aa1n02x5               g086(.a(new_n181), .b(new_n173), .c(new_n172), .d(new_n178), .o1(\s[16] ));
  nano23aa1n02x5               g087(.a(new_n134), .b(new_n139), .c(new_n140), .d(new_n135), .out0(new_n183));
  nand23aa1n03x5               g088(.a(new_n165), .b(new_n170), .c(new_n173), .o1(new_n184));
  nano32aa1n06x5               g089(.a(new_n184), .b(new_n183), .c(new_n128), .d(new_n130), .out0(new_n185));
  nand02aa1d10x5               g090(.a(new_n126), .b(new_n185), .o1(new_n186));
  nanp03aa1n02x5               g091(.a(new_n176), .b(new_n169), .c(new_n177), .o1(new_n187));
  oai112aa1n03x5               g092(.a(new_n179), .b(new_n162), .c(new_n161), .d(new_n152), .o1(new_n188));
  nor022aa1n04x5               g093(.a(new_n188), .b(new_n187), .o1(new_n189));
  oaoi03aa1n02x5               g094(.a(new_n174), .b(new_n175), .c(new_n168), .o1(new_n190));
  norb02aa1n06x5               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  aoai13aa1n06x5               g096(.a(new_n191), .b(new_n184), .c(new_n149), .d(new_n150), .o1(new_n192));
  inv000aa1n04x5               g097(.a(new_n192), .o1(new_n193));
  nand22aa1n12x5               g098(.a(new_n186), .b(new_n193), .o1(new_n194));
  xorc02aa1n12x5               g099(.a(\a[17] ), .b(\b[16] ), .out0(new_n195));
  nona22aa1n02x4               g100(.a(new_n190), .b(new_n189), .c(new_n195), .out0(new_n196));
  aoib12aa1n02x5               g101(.a(new_n196), .b(new_n151), .c(new_n184), .out0(new_n197));
  aoi022aa1n02x5               g102(.a(new_n194), .b(new_n195), .c(new_n186), .d(new_n197), .o1(\s[17] ));
  nor002aa1d32x5               g103(.a(\b[16] ), .b(\a[17] ), .o1(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  aoai13aa1n02x5               g105(.a(new_n195), .b(new_n192), .c(new_n126), .d(new_n185), .o1(new_n201));
  nor002aa1n16x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  nand02aa1d28x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  norb02aa1n06x4               g108(.a(new_n203), .b(new_n202), .out0(new_n204));
  xnbna2aa1n03x5               g109(.a(new_n204), .b(new_n201), .c(new_n200), .out0(\s[18] ));
  and002aa1n02x5               g110(.a(new_n195), .b(new_n204), .o(new_n206));
  aoai13aa1n03x5               g111(.a(new_n206), .b(new_n192), .c(new_n126), .d(new_n185), .o1(new_n207));
  oaoi03aa1n02x5               g112(.a(\a[18] ), .b(\b[17] ), .c(new_n200), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  nor042aa1d18x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nand02aa1n06x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  norb02aa1n12x5               g116(.a(new_n211), .b(new_n210), .out0(new_n212));
  xnbna2aa1n03x5               g117(.a(new_n212), .b(new_n207), .c(new_n209), .out0(\s[19] ));
  xnrc02aa1n02x5               g118(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g119(.a(new_n212), .b(new_n208), .c(new_n194), .d(new_n206), .o1(new_n215));
  nor042aa1n09x5               g120(.a(\b[19] ), .b(\a[20] ), .o1(new_n216));
  nand02aa1d28x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  norb02aa1n03x4               g122(.a(new_n217), .b(new_n216), .out0(new_n218));
  inv000aa1d42x5               g123(.a(\a[19] ), .o1(new_n219));
  inv000aa1d42x5               g124(.a(\b[18] ), .o1(new_n220));
  aboi22aa1n03x5               g125(.a(new_n216), .b(new_n217), .c(new_n219), .d(new_n220), .out0(new_n221));
  inv030aa1n03x5               g126(.a(new_n210), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n212), .o1(new_n223));
  aoai13aa1n02x5               g128(.a(new_n222), .b(new_n223), .c(new_n207), .d(new_n209), .o1(new_n224));
  aoi022aa1n03x5               g129(.a(new_n224), .b(new_n218), .c(new_n215), .d(new_n221), .o1(\s[20] ));
  nano32aa1n03x7               g130(.a(new_n223), .b(new_n195), .c(new_n218), .d(new_n204), .out0(new_n226));
  aoai13aa1n02x5               g131(.a(new_n226), .b(new_n192), .c(new_n126), .d(new_n185), .o1(new_n227));
  nanb03aa1n12x5               g132(.a(new_n216), .b(new_n217), .c(new_n211), .out0(new_n228));
  oai112aa1n06x5               g133(.a(new_n222), .b(new_n203), .c(new_n202), .d(new_n199), .o1(new_n229));
  aoi012aa1n12x5               g134(.a(new_n216), .b(new_n210), .c(new_n217), .o1(new_n230));
  oai012aa1d24x5               g135(.a(new_n230), .b(new_n229), .c(new_n228), .o1(new_n231));
  nor042aa1d18x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  nanp02aa1n06x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  norb02aa1d21x5               g138(.a(new_n233), .b(new_n232), .out0(new_n234));
  aoai13aa1n06x5               g139(.a(new_n234), .b(new_n231), .c(new_n194), .d(new_n226), .o1(new_n235));
  nano22aa1n12x5               g140(.a(new_n216), .b(new_n211), .c(new_n217), .out0(new_n236));
  oai012aa1n02x5               g141(.a(new_n203), .b(\b[18] ), .c(\a[19] ), .o1(new_n237));
  oab012aa1n02x5               g142(.a(new_n237), .b(new_n199), .c(new_n202), .out0(new_n238));
  inv020aa1n03x5               g143(.a(new_n230), .o1(new_n239));
  aoi112aa1n02x5               g144(.a(new_n239), .b(new_n234), .c(new_n238), .d(new_n236), .o1(new_n240));
  aobi12aa1n02x7               g145(.a(new_n235), .b(new_n240), .c(new_n227), .out0(\s[21] ));
  nor042aa1n06x5               g146(.a(\b[21] ), .b(\a[22] ), .o1(new_n242));
  nand02aa1n08x5               g147(.a(\b[21] ), .b(\a[22] ), .o1(new_n243));
  norb02aa1n02x5               g148(.a(new_n243), .b(new_n242), .out0(new_n244));
  aoib12aa1n02x5               g149(.a(new_n232), .b(new_n243), .c(new_n242), .out0(new_n245));
  inv000aa1d42x5               g150(.a(new_n231), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n232), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n234), .o1(new_n248));
  aoai13aa1n03x5               g153(.a(new_n247), .b(new_n248), .c(new_n227), .d(new_n246), .o1(new_n249));
  aoi022aa1n03x5               g154(.a(new_n249), .b(new_n244), .c(new_n235), .d(new_n245), .o1(\s[22] ));
  inv020aa1n02x5               g155(.a(new_n226), .o1(new_n251));
  nano22aa1n03x7               g156(.a(new_n251), .b(new_n234), .c(new_n244), .out0(new_n252));
  aoai13aa1n02x5               g157(.a(new_n252), .b(new_n192), .c(new_n126), .d(new_n185), .o1(new_n253));
  nano23aa1d15x5               g158(.a(new_n232), .b(new_n242), .c(new_n243), .d(new_n233), .out0(new_n254));
  aoi012aa1d18x5               g159(.a(new_n242), .b(new_n232), .c(new_n243), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  tech160nm_fiaoi012aa1n04x5   g161(.a(new_n256), .b(new_n231), .c(new_n254), .o1(new_n257));
  inv000aa1n02x5               g162(.a(new_n257), .o1(new_n258));
  xorc02aa1n12x5               g163(.a(\a[23] ), .b(\b[22] ), .out0(new_n259));
  aoai13aa1n06x5               g164(.a(new_n259), .b(new_n258), .c(new_n194), .d(new_n252), .o1(new_n260));
  aoi112aa1n02x5               g165(.a(new_n259), .b(new_n256), .c(new_n231), .d(new_n254), .o1(new_n261));
  aobi12aa1n02x7               g166(.a(new_n260), .b(new_n261), .c(new_n253), .out0(\s[23] ));
  tech160nm_fixorc02aa1n02p5x5 g167(.a(\a[24] ), .b(\b[23] ), .out0(new_n263));
  nor042aa1n06x5               g168(.a(\b[22] ), .b(\a[23] ), .o1(new_n264));
  norp02aa1n02x5               g169(.a(new_n263), .b(new_n264), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n264), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n259), .o1(new_n267));
  aoai13aa1n03x5               g172(.a(new_n266), .b(new_n267), .c(new_n253), .d(new_n257), .o1(new_n268));
  aoi022aa1n02x7               g173(.a(new_n268), .b(new_n263), .c(new_n260), .d(new_n265), .o1(\s[24] ));
  and002aa1n06x5               g174(.a(new_n263), .b(new_n259), .o(new_n270));
  nano22aa1n03x7               g175(.a(new_n251), .b(new_n270), .c(new_n254), .out0(new_n271));
  aoai13aa1n02x7               g176(.a(new_n271), .b(new_n192), .c(new_n126), .d(new_n185), .o1(new_n272));
  aoai13aa1n04x5               g177(.a(new_n254), .b(new_n239), .c(new_n238), .d(new_n236), .o1(new_n273));
  inv000aa1n06x5               g178(.a(new_n270), .o1(new_n274));
  oao003aa1n02x5               g179(.a(\a[24] ), .b(\b[23] ), .c(new_n266), .carry(new_n275));
  aoai13aa1n12x5               g180(.a(new_n275), .b(new_n274), .c(new_n273), .d(new_n255), .o1(new_n276));
  xorc02aa1n12x5               g181(.a(\a[25] ), .b(\b[24] ), .out0(new_n277));
  aoai13aa1n06x5               g182(.a(new_n277), .b(new_n276), .c(new_n194), .d(new_n271), .o1(new_n278));
  aoai13aa1n03x5               g183(.a(new_n270), .b(new_n256), .c(new_n231), .d(new_n254), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n277), .o1(new_n280));
  and003aa1n02x5               g185(.a(new_n279), .b(new_n280), .c(new_n275), .o(new_n281));
  aobi12aa1n03x7               g186(.a(new_n278), .b(new_n281), .c(new_n272), .out0(\s[25] ));
  tech160nm_fixorc02aa1n02p5x5 g187(.a(\a[26] ), .b(\b[25] ), .out0(new_n283));
  nor042aa1n03x5               g188(.a(\b[24] ), .b(\a[25] ), .o1(new_n284));
  norp02aa1n02x5               g189(.a(new_n283), .b(new_n284), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n276), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n284), .o1(new_n287));
  aoai13aa1n02x5               g192(.a(new_n287), .b(new_n280), .c(new_n272), .d(new_n286), .o1(new_n288));
  aoi022aa1n02x7               g193(.a(new_n288), .b(new_n283), .c(new_n278), .d(new_n285), .o1(\s[26] ));
  and002aa1n12x5               g194(.a(new_n283), .b(new_n277), .o(new_n290));
  nano32aa1n03x7               g195(.a(new_n251), .b(new_n290), .c(new_n254), .d(new_n270), .out0(new_n291));
  aoai13aa1n06x5               g196(.a(new_n291), .b(new_n192), .c(new_n126), .d(new_n185), .o1(new_n292));
  inv000aa1d42x5               g197(.a(new_n290), .o1(new_n293));
  oao003aa1n02x5               g198(.a(\a[26] ), .b(\b[25] ), .c(new_n287), .carry(new_n294));
  aoai13aa1n04x5               g199(.a(new_n294), .b(new_n293), .c(new_n279), .d(new_n275), .o1(new_n295));
  xorc02aa1n12x5               g200(.a(\a[27] ), .b(\b[26] ), .out0(new_n296));
  aoai13aa1n06x5               g201(.a(new_n296), .b(new_n295), .c(new_n194), .d(new_n291), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n294), .o1(new_n298));
  aoi112aa1n02x5               g203(.a(new_n296), .b(new_n298), .c(new_n276), .d(new_n290), .o1(new_n299));
  aobi12aa1n02x7               g204(.a(new_n297), .b(new_n299), .c(new_n292), .out0(\s[27] ));
  xorc02aa1n02x5               g205(.a(\a[28] ), .b(\b[27] ), .out0(new_n301));
  norp02aa1n02x5               g206(.a(\b[26] ), .b(\a[27] ), .o1(new_n302));
  norp02aa1n02x5               g207(.a(new_n301), .b(new_n302), .o1(new_n303));
  tech160nm_fiaoi012aa1n05x5   g208(.a(new_n298), .b(new_n276), .c(new_n290), .o1(new_n304));
  inv000aa1n03x5               g209(.a(new_n302), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n296), .o1(new_n306));
  aoai13aa1n03x5               g211(.a(new_n305), .b(new_n306), .c(new_n304), .d(new_n292), .o1(new_n307));
  aoi022aa1n03x5               g212(.a(new_n307), .b(new_n301), .c(new_n297), .d(new_n303), .o1(\s[28] ));
  and002aa1n02x5               g213(.a(new_n301), .b(new_n296), .o(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n295), .c(new_n194), .d(new_n291), .o1(new_n310));
  xorc02aa1n02x5               g215(.a(\a[29] ), .b(\b[28] ), .out0(new_n311));
  oao003aa1n02x5               g216(.a(\a[28] ), .b(\b[27] ), .c(new_n305), .carry(new_n312));
  norb02aa1n02x5               g217(.a(new_n312), .b(new_n311), .out0(new_n313));
  inv000aa1d42x5               g218(.a(new_n309), .o1(new_n314));
  aoai13aa1n03x5               g219(.a(new_n312), .b(new_n314), .c(new_n304), .d(new_n292), .o1(new_n315));
  aoi022aa1n03x5               g220(.a(new_n315), .b(new_n311), .c(new_n310), .d(new_n313), .o1(\s[29] ));
  inv000aa1d42x5               g221(.a(\a[2] ), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n99), .b(\b[1] ), .c(new_n317), .out0(\s[2] ));
  nano22aa1n02x4               g223(.a(new_n306), .b(new_n301), .c(new_n311), .out0(new_n319));
  aoai13aa1n02x5               g224(.a(new_n319), .b(new_n295), .c(new_n194), .d(new_n291), .o1(new_n320));
  xorc02aa1n02x5               g225(.a(\a[30] ), .b(\b[29] ), .out0(new_n321));
  inv000aa1d42x5               g226(.a(\a[29] ), .o1(new_n322));
  inv000aa1d42x5               g227(.a(\b[28] ), .o1(new_n323));
  oaib12aa1n02x5               g228(.a(new_n312), .b(\b[28] ), .c(new_n322), .out0(new_n324));
  oaoi13aa1n02x5               g229(.a(new_n321), .b(new_n324), .c(new_n322), .d(new_n323), .o1(new_n325));
  inv000aa1n02x5               g230(.a(new_n319), .o1(new_n326));
  oaib12aa1n02x5               g231(.a(new_n324), .b(new_n323), .c(\a[29] ), .out0(new_n327));
  aoai13aa1n03x5               g232(.a(new_n327), .b(new_n326), .c(new_n304), .d(new_n292), .o1(new_n328));
  aoi022aa1n03x5               g233(.a(new_n328), .b(new_n321), .c(new_n320), .d(new_n325), .o1(\s[30] ));
  nano32aa1d15x5               g234(.a(new_n306), .b(new_n321), .c(new_n301), .d(new_n311), .out0(new_n330));
  aoai13aa1n02x5               g235(.a(new_n330), .b(new_n295), .c(new_n194), .d(new_n291), .o1(new_n331));
  aoi022aa1n02x5               g236(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n332));
  norb02aa1n02x5               g237(.a(\b[30] ), .b(\a[31] ), .out0(new_n333));
  obai22aa1n02x7               g238(.a(\a[31] ), .b(\b[30] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n334));
  aoi112aa1n02x5               g239(.a(new_n334), .b(new_n333), .c(new_n324), .d(new_n332), .o1(new_n335));
  xorc02aa1n02x5               g240(.a(\a[31] ), .b(\b[30] ), .out0(new_n336));
  inv000aa1d42x5               g241(.a(new_n330), .o1(new_n337));
  norp02aa1n02x5               g242(.a(\b[29] ), .b(\a[30] ), .o1(new_n338));
  aoi012aa1n02x5               g243(.a(new_n338), .b(new_n324), .c(new_n332), .o1(new_n339));
  aoai13aa1n03x5               g244(.a(new_n339), .b(new_n337), .c(new_n304), .d(new_n292), .o1(new_n340));
  aoi022aa1n03x5               g245(.a(new_n340), .b(new_n336), .c(new_n331), .d(new_n335), .o1(\s[31] ));
  inv000aa1d42x5               g246(.a(\b[1] ), .o1(new_n342));
  aoi022aa1n02x5               g247(.a(new_n342), .b(new_n317), .c(\a[1] ), .d(\b[0] ), .o1(new_n343));
  oaib12aa1n02x5               g248(.a(new_n343), .b(new_n342), .c(\a[2] ), .out0(new_n344));
  aboi22aa1n03x5               g249(.a(new_n104), .b(new_n105), .c(new_n317), .d(new_n342), .out0(new_n345));
  aoi022aa1n02x5               g250(.a(new_n344), .b(new_n345), .c(new_n100), .d(new_n106), .o1(\s[3] ));
  aoai13aa1n02x5               g251(.a(new_n103), .b(new_n104), .c(new_n100), .d(new_n105), .o1(new_n347));
  aoi112aa1n02x5               g252(.a(new_n104), .b(new_n103), .c(new_n100), .d(new_n105), .o1(new_n348));
  norb02aa1n02x5               g253(.a(new_n347), .b(new_n348), .out0(\s[4] ));
  norb02aa1n02x5               g254(.a(new_n112), .b(new_n111), .out0(new_n350));
  xnbna2aa1n03x5               g255(.a(new_n350), .b(new_n107), .c(new_n108), .out0(\s[5] ));
  nanp02aa1n02x5               g256(.a(new_n107), .b(new_n108), .o1(new_n352));
  norb02aa1n02x5               g257(.a(new_n110), .b(new_n109), .out0(new_n353));
  aoai13aa1n02x5               g258(.a(new_n353), .b(new_n111), .c(new_n352), .d(new_n112), .o1(new_n354));
  aoi112aa1n02x5               g259(.a(new_n111), .b(new_n353), .c(new_n352), .d(new_n350), .o1(new_n355));
  norb02aa1n02x5               g260(.a(new_n354), .b(new_n355), .out0(\s[6] ));
  inv000aa1d42x5               g261(.a(new_n109), .o1(new_n357));
  xnbna2aa1n03x5               g262(.a(new_n117), .b(new_n354), .c(new_n357), .out0(\s[7] ));
  aob012aa1n02x5               g263(.a(new_n117), .b(new_n354), .c(new_n357), .out0(new_n359));
  oai012aa1n02x5               g264(.a(new_n359), .b(\b[6] ), .c(\a[7] ), .o1(new_n360));
  norp02aa1n02x5               g265(.a(new_n114), .b(new_n115), .o1(new_n361));
  aoi022aa1n02x5               g266(.a(new_n360), .b(new_n114), .c(new_n359), .d(new_n361), .o1(\s[8] ));
  aoi012aa1n02x5               g267(.a(new_n118), .b(new_n107), .c(new_n108), .o1(new_n363));
  aoi113aa1n02x5               g268(.a(new_n124), .b(new_n128), .c(new_n119), .d(new_n114), .e(new_n120), .o1(new_n364));
  aboi22aa1n03x5               g269(.a(new_n363), .b(new_n364), .c(new_n126), .d(new_n128), .out0(\s[9] ));
endmodule


