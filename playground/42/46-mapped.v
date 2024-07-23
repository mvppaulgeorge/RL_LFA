// Benchmark "adder" written by ABC on Thu Jul 18 09:54:46 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n254, new_n255, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n263, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n339, new_n340, new_n341, new_n342, new_n345, new_n347, new_n349,
    new_n350;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nanp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  xorc02aa1n12x5               g002(.a(\a[9] ), .b(\b[8] ), .out0(new_n98));
  oa0022aa1n06x5               g003(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n99));
  inv030aa1n02x5               g004(.a(new_n99), .o1(new_n100));
  xnrc02aa1n12x5               g005(.a(\b[2] ), .b(\a[3] ), .out0(new_n101));
  nanp02aa1n04x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nand02aa1d06x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  nor042aa1n03x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  oai012aa1n12x5               g009(.a(new_n102), .b(new_n104), .c(new_n103), .o1(new_n105));
  oab012aa1n09x5               g010(.a(new_n100), .b(new_n101), .c(new_n105), .out0(new_n106));
  and002aa1n12x5               g011(.a(\b[3] ), .b(\a[4] ), .o(new_n107));
  norp02aa1n12x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nanp02aa1n04x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  norb03aa1n12x5               g014(.a(new_n109), .b(new_n107), .c(new_n108), .out0(new_n110));
  xorc02aa1n12x5               g015(.a(\a[8] ), .b(\b[7] ), .out0(new_n111));
  nor022aa1n08x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nand42aa1n04x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nand42aa1n08x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nor002aa1n06x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nano23aa1n03x7               g020(.a(new_n115), .b(new_n112), .c(new_n113), .d(new_n114), .out0(new_n116));
  nand23aa1n06x5               g021(.a(new_n116), .b(new_n110), .c(new_n111), .o1(new_n117));
  norp02aa1n02x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  oaoi13aa1n04x5               g023(.a(new_n115), .b(new_n109), .c(new_n108), .d(new_n112), .o1(new_n119));
  aoi022aa1d24x5               g024(.a(\b[7] ), .b(\a[8] ), .c(\a[7] ), .d(\b[6] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(new_n120), .o1(new_n121));
  oab012aa1n12x5               g026(.a(new_n118), .b(new_n119), .c(new_n121), .out0(new_n122));
  oai112aa1n06x5               g027(.a(new_n122), .b(new_n98), .c(new_n106), .d(new_n117), .o1(new_n123));
  xnrc02aa1n12x5               g028(.a(\b[9] ), .b(\a[10] ), .out0(new_n124));
  xnbna2aa1n03x5               g029(.a(new_n124), .b(new_n123), .c(new_n97), .out0(\s[10] ));
  inv040aa1d32x5               g030(.a(\a[11] ), .o1(new_n126));
  inv040aa1d32x5               g031(.a(\b[10] ), .o1(new_n127));
  nand22aa1n12x5               g032(.a(new_n127), .b(new_n126), .o1(new_n128));
  nand02aa1d16x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nand22aa1n03x5               g034(.a(new_n128), .b(new_n129), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  aoai13aa1n02x5               g036(.a(new_n131), .b(new_n124), .c(new_n123), .d(new_n97), .o1(new_n132));
  nor002aa1n02x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nano22aa1n02x4               g038(.a(new_n133), .b(new_n131), .c(new_n129), .out0(new_n134));
  aoai13aa1n06x5               g039(.a(new_n134), .b(new_n124), .c(new_n123), .d(new_n97), .o1(new_n135));
  aobi12aa1n02x5               g040(.a(new_n135), .b(new_n132), .c(new_n130), .out0(\s[11] ));
  xorc02aa1n02x5               g041(.a(\a[12] ), .b(\b[11] ), .out0(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n137), .b(new_n135), .c(new_n128), .out0(\s[12] ));
  oai012aa1d24x5               g043(.a(new_n122), .b(new_n106), .c(new_n117), .o1(new_n139));
  nano23aa1n06x5               g044(.a(new_n130), .b(new_n124), .c(new_n137), .d(new_n98), .out0(new_n140));
  inv040aa1d32x5               g045(.a(\a[12] ), .o1(new_n141));
  inv000aa1d42x5               g046(.a(\b[11] ), .o1(new_n142));
  nand02aa1n02x5               g047(.a(new_n142), .b(new_n141), .o1(new_n143));
  nand42aa1n02x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nanp03aa1n03x5               g049(.a(new_n143), .b(new_n129), .c(new_n144), .o1(new_n145));
  oai022aa1n03x5               g050(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n146));
  nanp03aa1n03x5               g051(.a(new_n146), .b(new_n128), .c(new_n131), .o1(new_n147));
  tech160nm_fioaoi03aa1n03p5x5 g052(.a(new_n141), .b(new_n142), .c(new_n133), .o1(new_n148));
  oai012aa1n09x5               g053(.a(new_n148), .b(new_n147), .c(new_n145), .o1(new_n149));
  xorc02aa1n12x5               g054(.a(\a[13] ), .b(\b[12] ), .out0(new_n150));
  aoai13aa1n06x5               g055(.a(new_n150), .b(new_n149), .c(new_n139), .d(new_n140), .o1(new_n151));
  aoi112aa1n02x5               g056(.a(new_n150), .b(new_n149), .c(new_n139), .d(new_n140), .o1(new_n152));
  norb02aa1n02x5               g057(.a(new_n151), .b(new_n152), .out0(\s[13] ));
  inv000aa1d42x5               g058(.a(\a[13] ), .o1(new_n154));
  inv000aa1d42x5               g059(.a(\b[12] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(new_n155), .b(new_n154), .o1(new_n156));
  xorc02aa1n12x5               g061(.a(\a[14] ), .b(\b[13] ), .out0(new_n157));
  xnbna2aa1n03x5               g062(.a(new_n157), .b(new_n151), .c(new_n156), .out0(\s[14] ));
  oa0022aa1n06x5               g063(.a(\a[14] ), .b(\b[13] ), .c(\a[13] ), .d(\b[12] ), .o(new_n159));
  nand42aa1n02x5               g064(.a(new_n151), .b(new_n159), .o1(new_n160));
  nanp02aa1n09x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand42aa1d28x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  nor002aa1n16x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nano22aa1n02x4               g068(.a(new_n163), .b(new_n161), .c(new_n162), .out0(new_n164));
  nanp02aa1n03x5               g069(.a(new_n160), .b(new_n164), .o1(new_n165));
  inv000aa1d42x5               g070(.a(new_n163), .o1(new_n166));
  aoi022aa1n03x5               g071(.a(new_n160), .b(new_n161), .c(new_n166), .d(new_n162), .o1(new_n167));
  norb02aa1n02x7               g072(.a(new_n165), .b(new_n167), .out0(\s[15] ));
  inv000aa1n02x5               g073(.a(new_n164), .o1(new_n169));
  aoai13aa1n02x5               g074(.a(new_n166), .b(new_n169), .c(new_n151), .d(new_n159), .o1(new_n170));
  nor042aa1n09x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nand42aa1d28x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  norp02aa1n02x5               g078(.a(new_n173), .b(new_n163), .o1(new_n174));
  aoi022aa1n02x7               g079(.a(new_n170), .b(new_n173), .c(new_n165), .d(new_n174), .o1(\s[16] ));
  oai012aa1n02x5               g080(.a(new_n99), .b(new_n101), .c(new_n105), .o1(new_n176));
  nona23aa1n02x4               g081(.a(new_n114), .b(new_n113), .c(new_n115), .d(new_n112), .out0(new_n177));
  nano22aa1n03x5               g082(.a(new_n177), .b(new_n110), .c(new_n111), .out0(new_n178));
  oai022aa1n02x5               g083(.a(new_n119), .b(new_n121), .c(\b[7] ), .d(\a[8] ), .o1(new_n179));
  inv040aa1n09x5               g084(.a(new_n124), .o1(new_n180));
  nano22aa1n03x7               g085(.a(new_n130), .b(new_n143), .c(new_n144), .out0(new_n181));
  nano23aa1d15x5               g086(.a(new_n171), .b(new_n163), .c(new_n172), .d(new_n162), .out0(new_n182));
  nand23aa1d12x5               g087(.a(new_n182), .b(new_n150), .c(new_n157), .o1(new_n183));
  nano32aa1d15x5               g088(.a(new_n183), .b(new_n181), .c(new_n180), .d(new_n98), .out0(new_n184));
  aoai13aa1n06x5               g089(.a(new_n184), .b(new_n179), .c(new_n178), .d(new_n176), .o1(new_n185));
  oai012aa1n02x7               g090(.a(new_n161), .b(\b[14] ), .c(\a[15] ), .o1(new_n186));
  nanb03aa1n03x5               g091(.a(new_n171), .b(new_n172), .c(new_n162), .out0(new_n187));
  aoi012aa1n02x7               g092(.a(new_n171), .b(new_n163), .c(new_n172), .o1(new_n188));
  oai013aa1n03x5               g093(.a(new_n188), .b(new_n187), .c(new_n159), .d(new_n186), .o1(new_n189));
  aoib12aa1n06x5               g094(.a(new_n189), .b(new_n149), .c(new_n183), .out0(new_n190));
  xorc02aa1n12x5               g095(.a(\a[17] ), .b(\b[16] ), .out0(new_n191));
  xnbna2aa1n03x5               g096(.a(new_n191), .b(new_n185), .c(new_n190), .out0(\s[17] ));
  nor042aa1n04x5               g097(.a(\b[16] ), .b(\a[17] ), .o1(new_n193));
  inv000aa1d42x5               g098(.a(new_n193), .o1(new_n194));
  inv000aa1n06x5               g099(.a(new_n149), .o1(new_n195));
  oabi12aa1n18x5               g100(.a(new_n189), .b(new_n195), .c(new_n183), .out0(new_n196));
  aoai13aa1n06x5               g101(.a(new_n191), .b(new_n196), .c(new_n139), .d(new_n184), .o1(new_n197));
  nor042aa1n02x5               g102(.a(\b[17] ), .b(\a[18] ), .o1(new_n198));
  nand02aa1d08x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  norb02aa1n06x5               g104(.a(new_n199), .b(new_n198), .out0(new_n200));
  xnbna2aa1n03x5               g105(.a(new_n200), .b(new_n197), .c(new_n194), .out0(\s[18] ));
  nand42aa1n04x5               g106(.a(new_n185), .b(new_n190), .o1(new_n202));
  oai022aa1d24x5               g107(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n203));
  nand22aa1n09x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  oai012aa1n03x5               g109(.a(new_n199), .b(\b[18] ), .c(\a[19] ), .o1(new_n205));
  norb02aa1n02x5               g110(.a(new_n204), .b(new_n205), .out0(new_n206));
  aoai13aa1n06x5               g111(.a(new_n206), .b(new_n203), .c(new_n202), .d(new_n191), .o1(new_n207));
  inv040aa1d32x5               g112(.a(\a[19] ), .o1(new_n208));
  inv040aa1d32x5               g113(.a(\b[18] ), .o1(new_n209));
  nand02aa1d16x5               g114(.a(new_n209), .b(new_n208), .o1(new_n210));
  nanp02aa1n02x5               g115(.a(new_n210), .b(new_n204), .o1(new_n211));
  aoai13aa1n02x5               g116(.a(new_n199), .b(new_n203), .c(new_n202), .d(new_n191), .o1(new_n212));
  aobi12aa1n03x7               g117(.a(new_n207), .b(new_n212), .c(new_n211), .out0(\s[19] ));
  xnrc02aa1n02x5               g118(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g119(.a(new_n203), .o1(new_n215));
  inv000aa1n02x5               g120(.a(new_n206), .o1(new_n216));
  aoai13aa1n02x5               g121(.a(new_n210), .b(new_n216), .c(new_n197), .d(new_n215), .o1(new_n217));
  nor002aa1n16x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nand02aa1n06x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  aboi22aa1n03x5               g125(.a(new_n218), .b(new_n219), .c(new_n208), .d(new_n209), .out0(new_n221));
  aoi022aa1n03x5               g126(.a(new_n217), .b(new_n220), .c(new_n207), .d(new_n221), .o1(\s[20] ));
  inv000aa1n02x5               g127(.a(new_n218), .o1(new_n223));
  nano22aa1n03x7               g128(.a(new_n211), .b(new_n223), .c(new_n219), .out0(new_n224));
  nanp03aa1d12x5               g129(.a(new_n224), .b(new_n191), .c(new_n200), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n196), .c(new_n139), .d(new_n184), .o1(new_n227));
  nanb03aa1d24x5               g132(.a(new_n218), .b(new_n219), .c(new_n204), .out0(new_n228));
  nand23aa1d12x5               g133(.a(new_n203), .b(new_n210), .c(new_n199), .o1(new_n229));
  oaoi03aa1n09x5               g134(.a(\a[20] ), .b(\b[19] ), .c(new_n210), .o1(new_n230));
  oabi12aa1n18x5               g135(.a(new_n230), .b(new_n228), .c(new_n229), .out0(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  nor042aa1n09x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  nand42aa1n03x5               g138(.a(\b[20] ), .b(\a[21] ), .o1(new_n234));
  norb02aa1d21x5               g139(.a(new_n234), .b(new_n233), .out0(new_n235));
  xnbna2aa1n03x5               g140(.a(new_n235), .b(new_n227), .c(new_n232), .out0(\s[21] ));
  aoai13aa1n06x5               g141(.a(new_n235), .b(new_n231), .c(new_n202), .d(new_n226), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n233), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n235), .o1(new_n239));
  aoai13aa1n02x5               g144(.a(new_n238), .b(new_n239), .c(new_n227), .d(new_n232), .o1(new_n240));
  nor042aa1n02x5               g145(.a(\b[21] ), .b(\a[22] ), .o1(new_n241));
  nand42aa1n03x5               g146(.a(\b[21] ), .b(\a[22] ), .o1(new_n242));
  norb02aa1n02x5               g147(.a(new_n242), .b(new_n241), .out0(new_n243));
  aoib12aa1n02x5               g148(.a(new_n233), .b(new_n242), .c(new_n241), .out0(new_n244));
  aoi022aa1n03x5               g149(.a(new_n240), .b(new_n243), .c(new_n237), .d(new_n244), .o1(\s[22] ));
  nano23aa1n09x5               g150(.a(new_n233), .b(new_n241), .c(new_n242), .d(new_n234), .out0(new_n246));
  norb02aa1n06x5               g151(.a(new_n246), .b(new_n225), .out0(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n196), .c(new_n139), .d(new_n184), .o1(new_n248));
  tech160nm_fioai012aa1n03p5x5 g153(.a(new_n242), .b(new_n241), .c(new_n233), .o1(new_n249));
  inv030aa1n02x5               g154(.a(new_n249), .o1(new_n250));
  aoi012aa1n02x5               g155(.a(new_n250), .b(new_n231), .c(new_n246), .o1(new_n251));
  xorc02aa1n02x5               g156(.a(\a[23] ), .b(\b[22] ), .out0(new_n252));
  xnbna2aa1n03x5               g157(.a(new_n252), .b(new_n248), .c(new_n251), .out0(\s[23] ));
  aob012aa1n03x5               g158(.a(new_n252), .b(new_n248), .c(new_n251), .out0(new_n254));
  inv000aa1d42x5               g159(.a(\a[23] ), .o1(new_n255));
  inv000aa1d42x5               g160(.a(\b[22] ), .o1(new_n256));
  nanp02aa1n02x5               g161(.a(new_n256), .b(new_n255), .o1(new_n257));
  and002aa1n02x5               g162(.a(\b[22] ), .b(\a[23] ), .o(new_n258));
  aoai13aa1n02x7               g163(.a(new_n257), .b(new_n258), .c(new_n248), .d(new_n251), .o1(new_n259));
  xorc02aa1n02x5               g164(.a(\a[24] ), .b(\b[23] ), .out0(new_n260));
  norb02aa1n02x5               g165(.a(new_n257), .b(new_n260), .out0(new_n261));
  aoi022aa1n03x5               g166(.a(new_n259), .b(new_n260), .c(new_n254), .d(new_n261), .o1(\s[24] ));
  inv000aa1d42x5               g167(.a(\a[24] ), .o1(new_n263));
  xroi22aa1d04x5               g168(.a(new_n255), .b(\b[22] ), .c(new_n263), .d(\b[23] ), .out0(new_n264));
  nano22aa1n03x7               g169(.a(new_n225), .b(new_n264), .c(new_n246), .out0(new_n265));
  aoai13aa1n06x5               g170(.a(new_n265), .b(new_n196), .c(new_n139), .d(new_n184), .o1(new_n266));
  nand42aa1n03x5               g171(.a(\b[23] ), .b(\a[24] ), .o1(new_n267));
  nano22aa1n02x4               g172(.a(new_n218), .b(new_n204), .c(new_n219), .out0(new_n268));
  oab012aa1n06x5               g173(.a(new_n205), .b(new_n193), .c(new_n198), .out0(new_n269));
  aoai13aa1n06x5               g174(.a(new_n246), .b(new_n230), .c(new_n269), .d(new_n268), .o1(new_n270));
  aboi22aa1n03x5               g175(.a(\b[23] ), .b(new_n263), .c(new_n255), .d(new_n256), .out0(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n258), .c(new_n270), .d(new_n249), .o1(new_n272));
  nanp02aa1n03x5               g177(.a(new_n272), .b(new_n267), .o1(new_n273));
  nor002aa1d32x5               g178(.a(\b[24] ), .b(\a[25] ), .o1(new_n274));
  and002aa1n02x5               g179(.a(\b[24] ), .b(\a[25] ), .o(new_n275));
  norp02aa1n02x5               g180(.a(new_n275), .b(new_n274), .o1(new_n276));
  xnbna2aa1n03x5               g181(.a(new_n276), .b(new_n266), .c(new_n273), .out0(\s[25] ));
  aob012aa1n03x5               g182(.a(new_n276), .b(new_n266), .c(new_n273), .out0(new_n278));
  inv040aa1n03x5               g183(.a(new_n274), .o1(new_n279));
  aoai13aa1n03x5               g184(.a(new_n279), .b(new_n275), .c(new_n266), .d(new_n273), .o1(new_n280));
  xorc02aa1n02x5               g185(.a(\a[26] ), .b(\b[25] ), .out0(new_n281));
  norp02aa1n02x5               g186(.a(new_n281), .b(new_n274), .o1(new_n282));
  aoi022aa1n03x5               g187(.a(new_n280), .b(new_n281), .c(new_n278), .d(new_n282), .o1(\s[26] ));
  nanp02aa1n02x5               g188(.a(\b[25] ), .b(\a[26] ), .o1(new_n284));
  inv000aa1d42x5               g189(.a(\a[26] ), .o1(new_n285));
  inv000aa1d42x5               g190(.a(\b[25] ), .o1(new_n286));
  ao0022aa1n03x5               g191(.a(new_n285), .b(new_n286), .c(\a[25] ), .d(\b[24] ), .o(new_n287));
  nano32aa1n09x5               g192(.a(new_n287), .b(new_n284), .c(new_n279), .d(new_n267), .out0(new_n288));
  nano32aa1n03x7               g193(.a(new_n225), .b(new_n288), .c(new_n246), .d(new_n264), .out0(new_n289));
  aoai13aa1n06x5               g194(.a(new_n289), .b(new_n196), .c(new_n139), .d(new_n184), .o1(new_n290));
  aobi12aa1n09x5               g195(.a(new_n289), .b(new_n185), .c(new_n190), .out0(new_n291));
  inv000aa1d42x5               g196(.a(new_n258), .o1(new_n292));
  aoai13aa1n06x5               g197(.a(new_n292), .b(new_n250), .c(new_n231), .d(new_n246), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n288), .o1(new_n294));
  oaoi03aa1n02x5               g199(.a(new_n285), .b(new_n286), .c(new_n274), .o1(new_n295));
  aoai13aa1n06x5               g200(.a(new_n295), .b(new_n294), .c(new_n293), .d(new_n271), .o1(new_n296));
  xorc02aa1n12x5               g201(.a(\a[27] ), .b(\b[26] ), .out0(new_n297));
  tech160nm_fioai012aa1n05x5   g202(.a(new_n297), .b(new_n296), .c(new_n291), .o1(new_n298));
  inv000aa1n02x5               g203(.a(new_n295), .o1(new_n299));
  aoi112aa1n02x5               g204(.a(new_n297), .b(new_n299), .c(new_n272), .d(new_n288), .o1(new_n300));
  aobi12aa1n02x7               g205(.a(new_n298), .b(new_n300), .c(new_n290), .out0(\s[27] ));
  aoi012aa1n06x5               g206(.a(new_n299), .b(new_n272), .c(new_n288), .o1(new_n302));
  nor042aa1n03x5               g207(.a(\b[26] ), .b(\a[27] ), .o1(new_n303));
  inv000aa1n03x5               g208(.a(new_n303), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n297), .o1(new_n305));
  aoai13aa1n02x7               g210(.a(new_n304), .b(new_n305), .c(new_n302), .d(new_n290), .o1(new_n306));
  xorc02aa1n02x5               g211(.a(\a[28] ), .b(\b[27] ), .out0(new_n307));
  norp02aa1n02x5               g212(.a(new_n307), .b(new_n303), .o1(new_n308));
  aoi022aa1n02x7               g213(.a(new_n306), .b(new_n307), .c(new_n298), .d(new_n308), .o1(\s[28] ));
  and002aa1n02x5               g214(.a(new_n307), .b(new_n297), .o(new_n310));
  oai012aa1n03x5               g215(.a(new_n310), .b(new_n296), .c(new_n291), .o1(new_n311));
  inv000aa1d42x5               g216(.a(new_n310), .o1(new_n312));
  oao003aa1n03x5               g217(.a(\a[28] ), .b(\b[27] ), .c(new_n304), .carry(new_n313));
  aoai13aa1n02x7               g218(.a(new_n313), .b(new_n312), .c(new_n302), .d(new_n290), .o1(new_n314));
  xorc02aa1n02x5               g219(.a(\a[29] ), .b(\b[28] ), .out0(new_n315));
  norb02aa1n02x5               g220(.a(new_n313), .b(new_n315), .out0(new_n316));
  aoi022aa1n03x5               g221(.a(new_n314), .b(new_n315), .c(new_n311), .d(new_n316), .o1(\s[29] ));
  xorb03aa1n02x5               g222(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g223(.a(new_n305), .b(new_n307), .c(new_n315), .out0(new_n319));
  oai012aa1n03x5               g224(.a(new_n319), .b(new_n296), .c(new_n291), .o1(new_n320));
  inv000aa1n02x5               g225(.a(new_n319), .o1(new_n321));
  oaoi03aa1n02x5               g226(.a(\a[29] ), .b(\b[28] ), .c(new_n313), .o1(new_n322));
  inv000aa1n03x5               g227(.a(new_n322), .o1(new_n323));
  aoai13aa1n02x7               g228(.a(new_n323), .b(new_n321), .c(new_n302), .d(new_n290), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[30] ), .b(\b[29] ), .out0(new_n325));
  and002aa1n02x5               g230(.a(\b[28] ), .b(\a[29] ), .o(new_n326));
  oabi12aa1n02x5               g231(.a(new_n325), .b(\a[29] ), .c(\b[28] ), .out0(new_n327));
  oab012aa1n02x4               g232(.a(new_n327), .b(new_n313), .c(new_n326), .out0(new_n328));
  aoi022aa1n03x5               g233(.a(new_n324), .b(new_n325), .c(new_n320), .d(new_n328), .o1(\s[30] ));
  nano32aa1n02x4               g234(.a(new_n305), .b(new_n325), .c(new_n307), .d(new_n315), .out0(new_n330));
  oai012aa1n03x5               g235(.a(new_n330), .b(new_n296), .c(new_n291), .o1(new_n331));
  xorc02aa1n02x5               g236(.a(\a[31] ), .b(\b[30] ), .out0(new_n332));
  oao003aa1n02x5               g237(.a(\a[30] ), .b(\b[29] ), .c(new_n323), .carry(new_n333));
  norb02aa1n02x5               g238(.a(new_n333), .b(new_n332), .out0(new_n334));
  inv000aa1n02x5               g239(.a(new_n330), .o1(new_n335));
  aoai13aa1n02x7               g240(.a(new_n333), .b(new_n335), .c(new_n302), .d(new_n290), .o1(new_n336));
  aoi022aa1n03x5               g241(.a(new_n336), .b(new_n332), .c(new_n331), .d(new_n334), .o1(\s[31] ));
  xnrb03aa1n02x5               g242(.a(new_n105), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  norp02aa1n02x5               g243(.a(\b[3] ), .b(\a[4] ), .o1(new_n339));
  oai022aa1n02x5               g244(.a(new_n107), .b(new_n339), .c(\b[2] ), .d(\a[3] ), .o1(new_n340));
  oab012aa1n02x4               g245(.a(new_n340), .b(new_n101), .c(new_n105), .out0(new_n341));
  oaoi13aa1n04x5               g246(.a(new_n107), .b(new_n99), .c(new_n101), .d(new_n105), .o1(new_n342));
  oaoi13aa1n02x5               g247(.a(new_n341), .b(new_n342), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  xorb03aa1n02x5               g248(.a(new_n342), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g249(.a(new_n112), .b(new_n342), .c(new_n113), .o1(new_n345));
  xnrb03aa1n02x5               g250(.a(new_n345), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fioaoi03aa1n03p5x5 g251(.a(\a[6] ), .b(\b[5] ), .c(new_n345), .o1(new_n347));
  xorb03aa1n02x5               g252(.a(new_n347), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoai13aa1n02x5               g253(.a(new_n111), .b(new_n115), .c(new_n347), .d(new_n114), .o1(new_n349));
  aoi112aa1n02x5               g254(.a(new_n115), .b(new_n111), .c(new_n347), .d(new_n114), .o1(new_n350));
  norb02aa1n03x4               g255(.a(new_n349), .b(new_n350), .out0(\s[8] ));
  xorb03aa1n02x5               g256(.a(new_n139), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


