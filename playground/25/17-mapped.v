// Benchmark "adder" written by ABC on Thu Jul 18 00:54:21 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n162, new_n163,
    new_n164, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n339, new_n342, new_n344, new_n345, new_n347;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n04x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor002aa1n04x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand22aa1n06x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nand22aa1n06x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  aoi012aa1n06x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  xorc02aa1n02x5               g006(.a(\a[4] ), .b(\b[3] ), .out0(new_n102));
  nor042aa1n06x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand42aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  norb02aa1n02x5               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  nanb03aa1n06x5               g010(.a(new_n101), .b(new_n102), .c(new_n105), .out0(new_n106));
  inv000aa1d42x5               g011(.a(\a[4] ), .o1(new_n107));
  inv000aa1d42x5               g012(.a(\b[3] ), .o1(new_n108));
  oaoi03aa1n09x5               g013(.a(new_n107), .b(new_n108), .c(new_n103), .o1(new_n109));
  nor002aa1n03x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  nand42aa1n02x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nanb02aa1n02x5               g016(.a(new_n110), .b(new_n111), .out0(new_n112));
  xnrc02aa1n02x5               g017(.a(\b[4] ), .b(\a[5] ), .out0(new_n113));
  nor042aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nand22aa1n03x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  norp02aa1n06x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nand22aa1n03x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nano23aa1n06x5               g022(.a(new_n114), .b(new_n116), .c(new_n117), .d(new_n115), .out0(new_n118));
  nona22aa1n02x4               g023(.a(new_n118), .b(new_n113), .c(new_n112), .out0(new_n119));
  tech160nm_fiao0012aa1n02p5x5 g024(.a(new_n114), .b(new_n116), .c(new_n115), .o(new_n120));
  inv000aa1d42x5               g025(.a(\a[5] ), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\b[4] ), .o1(new_n122));
  aoai13aa1n04x5               g027(.a(new_n111), .b(new_n110), .c(new_n121), .d(new_n122), .o1(new_n123));
  aoib12aa1n12x5               g028(.a(new_n120), .b(new_n118), .c(new_n123), .out0(new_n124));
  aoai13aa1n12x5               g029(.a(new_n124), .b(new_n119), .c(new_n106), .d(new_n109), .o1(new_n125));
  nand42aa1n03x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  aoi012aa1n02x5               g031(.a(new_n97), .b(new_n125), .c(new_n126), .o1(new_n127));
  xnrb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor042aa1n04x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand42aa1n03x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nano23aa1n09x5               g035(.a(new_n129), .b(new_n97), .c(new_n126), .d(new_n130), .out0(new_n131));
  tech160nm_fioai012aa1n05x5   g036(.a(new_n130), .b(new_n97), .c(new_n129), .o1(new_n132));
  inv040aa1n02x5               g037(.a(new_n132), .o1(new_n133));
  nor002aa1n03x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand02aa1d04x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  aoai13aa1n06x5               g041(.a(new_n136), .b(new_n133), .c(new_n125), .d(new_n131), .o1(new_n137));
  aoi112aa1n02x5               g042(.a(new_n136), .b(new_n133), .c(new_n125), .d(new_n131), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n137), .b(new_n138), .out0(\s[11] ));
  tech160nm_fioai012aa1n03p5x5 g044(.a(new_n137), .b(\b[10] ), .c(\a[11] ), .o1(new_n140));
  xorb03aa1n02x5               g045(.a(new_n140), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nanp02aa1n02x5               g046(.a(new_n108), .b(new_n107), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(\b[3] ), .b(\a[4] ), .o1(new_n143));
  nanp02aa1n02x5               g048(.a(new_n142), .b(new_n143), .o1(new_n144));
  inv000aa1d42x5               g049(.a(\a[3] ), .o1(new_n145));
  inv000aa1d42x5               g050(.a(\b[2] ), .o1(new_n146));
  nanp02aa1n02x5               g051(.a(new_n146), .b(new_n145), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(new_n147), .b(new_n104), .o1(new_n148));
  norp03aa1n02x5               g053(.a(new_n101), .b(new_n144), .c(new_n148), .o1(new_n149));
  inv000aa1n02x5               g054(.a(new_n109), .o1(new_n150));
  nona23aa1n09x5               g055(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n151));
  nor043aa1n02x5               g056(.a(new_n151), .b(new_n113), .c(new_n112), .o1(new_n152));
  oai012aa1n02x7               g057(.a(new_n152), .b(new_n149), .c(new_n150), .o1(new_n153));
  nor002aa1n03x5               g058(.a(\b[11] ), .b(\a[12] ), .o1(new_n154));
  nanp02aa1n04x5               g059(.a(\b[11] ), .b(\a[12] ), .o1(new_n155));
  nano23aa1n06x5               g060(.a(new_n134), .b(new_n154), .c(new_n155), .d(new_n135), .out0(new_n156));
  nand22aa1n12x5               g061(.a(new_n156), .b(new_n131), .o1(new_n157));
  aoi012aa1n02x7               g062(.a(new_n154), .b(new_n134), .c(new_n155), .o1(new_n158));
  aobi12aa1n06x5               g063(.a(new_n158), .b(new_n156), .c(new_n133), .out0(new_n159));
  aoai13aa1n02x5               g064(.a(new_n159), .b(new_n157), .c(new_n153), .d(new_n124), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n06x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nand42aa1n03x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  aoi012aa1n02x5               g068(.a(new_n162), .b(new_n160), .c(new_n163), .o1(new_n164));
  xnrb03aa1n02x5               g069(.a(new_n164), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  inv000aa1d42x5               g070(.a(new_n157), .o1(new_n166));
  norp02aa1n06x5               g071(.a(\b[13] ), .b(\a[14] ), .o1(new_n167));
  nand42aa1n02x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nano23aa1n03x7               g073(.a(new_n162), .b(new_n167), .c(new_n168), .d(new_n163), .out0(new_n169));
  nona23aa1n03x5               g074(.a(new_n168), .b(new_n163), .c(new_n162), .d(new_n167), .out0(new_n170));
  oai012aa1n02x5               g075(.a(new_n168), .b(new_n167), .c(new_n162), .o1(new_n171));
  oai012aa1n02x5               g076(.a(new_n171), .b(new_n159), .c(new_n170), .o1(new_n172));
  aoi013aa1n06x4               g077(.a(new_n172), .b(new_n125), .c(new_n166), .d(new_n169), .o1(new_n173));
  nor042aa1n09x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  inv000aa1d42x5               g079(.a(new_n174), .o1(new_n175));
  nanp02aa1n02x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  xnbna2aa1n03x5               g081(.a(new_n173), .b(new_n176), .c(new_n175), .out0(\s[15] ));
  nanb02aa1n02x5               g082(.a(new_n174), .b(new_n176), .out0(new_n178));
  nor042aa1n03x5               g083(.a(\b[15] ), .b(\a[16] ), .o1(new_n179));
  nanp02aa1n02x5               g084(.a(\b[15] ), .b(\a[16] ), .o1(new_n180));
  nanb02aa1n02x5               g085(.a(new_n179), .b(new_n180), .out0(new_n181));
  oaoi13aa1n02x5               g086(.a(new_n181), .b(new_n175), .c(new_n173), .d(new_n178), .o1(new_n182));
  oai112aa1n02x5               g087(.a(new_n181), .b(new_n175), .c(new_n173), .d(new_n178), .o1(new_n183));
  norb02aa1n02x5               g088(.a(new_n183), .b(new_n182), .out0(\s[16] ));
  oai013aa1n06x5               g089(.a(new_n109), .b(new_n101), .c(new_n144), .d(new_n148), .o1(new_n185));
  oabi12aa1n02x5               g090(.a(new_n120), .b(new_n151), .c(new_n123), .out0(new_n186));
  nano23aa1n03x7               g091(.a(new_n174), .b(new_n179), .c(new_n180), .d(new_n176), .out0(new_n187));
  nano22aa1n03x7               g092(.a(new_n157), .b(new_n169), .c(new_n187), .out0(new_n188));
  aoai13aa1n06x5               g093(.a(new_n188), .b(new_n186), .c(new_n185), .d(new_n152), .o1(new_n189));
  nona23aa1n03x5               g094(.a(new_n155), .b(new_n135), .c(new_n134), .d(new_n154), .out0(new_n190));
  tech160nm_fioai012aa1n04x5   g095(.a(new_n158), .b(new_n190), .c(new_n132), .o1(new_n191));
  nona23aa1n03x5               g096(.a(new_n180), .b(new_n176), .c(new_n174), .d(new_n179), .out0(new_n192));
  norp02aa1n06x5               g097(.a(new_n192), .b(new_n170), .o1(new_n193));
  aoi012aa1n02x5               g098(.a(new_n179), .b(new_n174), .c(new_n180), .o1(new_n194));
  oai012aa1n04x7               g099(.a(new_n194), .b(new_n192), .c(new_n171), .o1(new_n195));
  aoi012aa1n12x5               g100(.a(new_n195), .b(new_n191), .c(new_n193), .o1(new_n196));
  xorc02aa1n12x5               g101(.a(\a[17] ), .b(\b[16] ), .out0(new_n197));
  xnbna2aa1n03x5               g102(.a(new_n197), .b(new_n189), .c(new_n196), .out0(\s[17] ));
  inv040aa1d28x5               g103(.a(\a[17] ), .o1(new_n199));
  inv040aa1d32x5               g104(.a(\b[16] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(new_n200), .b(new_n199), .o1(new_n201));
  inv000aa1n02x5               g106(.a(new_n193), .o1(new_n202));
  oabi12aa1n06x5               g107(.a(new_n195), .b(new_n202), .c(new_n159), .out0(new_n203));
  aoai13aa1n02x5               g108(.a(new_n197), .b(new_n203), .c(new_n125), .d(new_n188), .o1(new_n204));
  nor022aa1n16x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  nand42aa1d28x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  nanb02aa1n06x5               g111(.a(new_n205), .b(new_n206), .out0(new_n207));
  xobna2aa1n03x5               g112(.a(new_n207), .b(new_n204), .c(new_n201), .out0(\s[18] ));
  norb02aa1n03x5               g113(.a(new_n197), .b(new_n207), .out0(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  aoai13aa1n12x5               g115(.a(new_n206), .b(new_n205), .c(new_n199), .d(new_n200), .o1(new_n211));
  aoai13aa1n06x5               g116(.a(new_n211), .b(new_n210), .c(new_n189), .d(new_n196), .o1(new_n212));
  xorb03aa1n03x5               g117(.a(new_n212), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g118(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  nona23aa1n02x4               g121(.a(new_n131), .b(new_n187), .c(new_n170), .d(new_n190), .out0(new_n217));
  aoai13aa1n06x5               g122(.a(new_n196), .b(new_n217), .c(new_n153), .d(new_n124), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n211), .o1(new_n219));
  nand42aa1n06x5               g124(.a(\b[18] ), .b(\a[19] ), .o1(new_n220));
  nanb02aa1d24x5               g125(.a(new_n215), .b(new_n220), .out0(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  aoai13aa1n03x5               g127(.a(new_n222), .b(new_n219), .c(new_n218), .d(new_n209), .o1(new_n223));
  nor042aa1n06x5               g128(.a(\b[19] ), .b(\a[20] ), .o1(new_n224));
  nand02aa1n08x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  nanb02aa1n09x5               g130(.a(new_n224), .b(new_n225), .out0(new_n226));
  aoi012aa1n02x7               g131(.a(new_n226), .b(new_n223), .c(new_n216), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n226), .o1(new_n228));
  aoi112aa1n03x4               g133(.a(new_n215), .b(new_n228), .c(new_n212), .d(new_n222), .o1(new_n229));
  norp02aa1n03x5               g134(.a(new_n227), .b(new_n229), .o1(\s[20] ));
  nona23aa1d16x5               g135(.a(new_n197), .b(new_n228), .c(new_n221), .d(new_n207), .out0(new_n231));
  nona23aa1n09x5               g136(.a(new_n225), .b(new_n220), .c(new_n215), .d(new_n224), .out0(new_n232));
  tech160nm_fiaoi012aa1n03p5x5 g137(.a(new_n224), .b(new_n215), .c(new_n225), .o1(new_n233));
  oai012aa1d24x5               g138(.a(new_n233), .b(new_n232), .c(new_n211), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  aoai13aa1n04x5               g140(.a(new_n235), .b(new_n231), .c(new_n189), .d(new_n196), .o1(new_n236));
  xorb03aa1n02x5               g141(.a(new_n236), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1d32x5               g142(.a(\b[20] ), .b(\a[21] ), .o1(new_n238));
  inv040aa1n08x5               g143(.a(new_n238), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n231), .o1(new_n240));
  nand42aa1n06x5               g145(.a(\b[20] ), .b(\a[21] ), .o1(new_n241));
  nanb02aa1n02x5               g146(.a(new_n238), .b(new_n241), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  aoai13aa1n03x5               g148(.a(new_n243), .b(new_n234), .c(new_n218), .d(new_n240), .o1(new_n244));
  norp02aa1n09x5               g149(.a(\b[21] ), .b(\a[22] ), .o1(new_n245));
  nanp02aa1n04x5               g150(.a(\b[21] ), .b(\a[22] ), .o1(new_n246));
  nanb02aa1n02x5               g151(.a(new_n245), .b(new_n246), .out0(new_n247));
  aoi012aa1n03x5               g152(.a(new_n247), .b(new_n244), .c(new_n239), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n247), .o1(new_n249));
  aoi112aa1n03x5               g154(.a(new_n238), .b(new_n249), .c(new_n236), .d(new_n243), .o1(new_n250));
  norp02aa1n03x5               g155(.a(new_n248), .b(new_n250), .o1(\s[22] ));
  nano23aa1n02x4               g156(.a(new_n215), .b(new_n224), .c(new_n225), .d(new_n220), .out0(new_n252));
  nona23aa1n02x4               g157(.a(new_n246), .b(new_n241), .c(new_n238), .d(new_n245), .out0(new_n253));
  nano23aa1n03x7               g158(.a(new_n207), .b(new_n253), .c(new_n252), .d(new_n197), .out0(new_n254));
  inv000aa1n02x5               g159(.a(new_n254), .o1(new_n255));
  nano23aa1n03x7               g160(.a(new_n238), .b(new_n245), .c(new_n246), .d(new_n241), .out0(new_n256));
  tech160nm_fioaoi03aa1n03p5x5 g161(.a(\a[22] ), .b(\b[21] ), .c(new_n239), .o1(new_n257));
  aoi012aa1n12x5               g162(.a(new_n257), .b(new_n234), .c(new_n256), .o1(new_n258));
  aoai13aa1n06x5               g163(.a(new_n258), .b(new_n255), .c(new_n189), .d(new_n196), .o1(new_n259));
  xorb03aa1n02x5               g164(.a(new_n259), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  inv000aa1n02x5               g166(.a(new_n261), .o1(new_n262));
  inv040aa1d30x5               g167(.a(new_n258), .o1(new_n263));
  xorc02aa1n12x5               g168(.a(\a[23] ), .b(\b[22] ), .out0(new_n264));
  aoai13aa1n03x5               g169(.a(new_n264), .b(new_n263), .c(new_n218), .d(new_n254), .o1(new_n265));
  xnrc02aa1n02x5               g170(.a(\b[23] ), .b(\a[24] ), .out0(new_n266));
  tech160nm_fiaoi012aa1n02p5x5 g171(.a(new_n266), .b(new_n265), .c(new_n262), .o1(new_n267));
  tech160nm_fixorc02aa1n04x5   g172(.a(\a[24] ), .b(\b[23] ), .out0(new_n268));
  aoi112aa1n03x4               g173(.a(new_n261), .b(new_n268), .c(new_n259), .d(new_n264), .o1(new_n269));
  nor002aa1n02x5               g174(.a(new_n267), .b(new_n269), .o1(\s[24] ));
  nanp03aa1n06x5               g175(.a(new_n256), .b(new_n264), .c(new_n268), .o1(new_n271));
  nor002aa1n02x5               g176(.a(new_n231), .b(new_n271), .o1(new_n272));
  inv020aa1n02x5               g177(.a(new_n272), .o1(new_n273));
  xnrc02aa1n02x5               g178(.a(\b[22] ), .b(\a[23] ), .out0(new_n274));
  nor043aa1n02x5               g179(.a(new_n253), .b(new_n274), .c(new_n266), .o1(new_n275));
  nand22aa1n12x5               g180(.a(new_n234), .b(new_n275), .o1(new_n276));
  oaoi03aa1n02x5               g181(.a(\a[24] ), .b(\b[23] ), .c(new_n262), .o1(new_n277));
  aoi013aa1n09x5               g182(.a(new_n277), .b(new_n257), .c(new_n264), .d(new_n268), .o1(new_n278));
  nand02aa1d16x5               g183(.a(new_n276), .b(new_n278), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n279), .o1(new_n280));
  aoai13aa1n04x5               g185(.a(new_n280), .b(new_n273), .c(new_n189), .d(new_n196), .o1(new_n281));
  xorb03aa1n02x5               g186(.a(new_n281), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g187(.a(\b[24] ), .b(\a[25] ), .o1(new_n283));
  inv000aa1n02x5               g188(.a(new_n283), .o1(new_n284));
  tech160nm_fixorc02aa1n02p5x5 g189(.a(\a[25] ), .b(\b[24] ), .out0(new_n285));
  aoai13aa1n03x5               g190(.a(new_n285), .b(new_n279), .c(new_n218), .d(new_n272), .o1(new_n286));
  xorc02aa1n12x5               g191(.a(\a[26] ), .b(\b[25] ), .out0(new_n287));
  inv000aa1d42x5               g192(.a(new_n287), .o1(new_n288));
  aoi012aa1n03x5               g193(.a(new_n288), .b(new_n286), .c(new_n284), .o1(new_n289));
  aoi112aa1n03x4               g194(.a(new_n283), .b(new_n287), .c(new_n281), .d(new_n285), .o1(new_n290));
  norp02aa1n03x5               g195(.a(new_n289), .b(new_n290), .o1(\s[26] ));
  and002aa1n18x5               g196(.a(new_n287), .b(new_n285), .o(new_n292));
  norb03aa1n12x5               g197(.a(new_n292), .b(new_n231), .c(new_n271), .out0(new_n293));
  aoai13aa1n06x5               g198(.a(new_n293), .b(new_n203), .c(new_n125), .d(new_n188), .o1(new_n294));
  oao003aa1n02x5               g199(.a(\a[26] ), .b(\b[25] ), .c(new_n284), .carry(new_n295));
  aobi12aa1n18x5               g200(.a(new_n295), .b(new_n279), .c(new_n292), .out0(new_n296));
  xorc02aa1n12x5               g201(.a(\a[27] ), .b(\b[26] ), .out0(new_n297));
  xnbna2aa1n03x5               g202(.a(new_n297), .b(new_n294), .c(new_n296), .out0(\s[27] ));
  norp02aa1n02x5               g203(.a(\b[26] ), .b(\a[27] ), .o1(new_n299));
  inv040aa1n03x5               g204(.a(new_n299), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n292), .o1(new_n301));
  aoai13aa1n06x5               g206(.a(new_n295), .b(new_n301), .c(new_n276), .d(new_n278), .o1(new_n302));
  aoai13aa1n06x5               g207(.a(new_n297), .b(new_n302), .c(new_n218), .d(new_n293), .o1(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[27] ), .b(\a[28] ), .out0(new_n304));
  tech160nm_fiaoi012aa1n02p5x5 g209(.a(new_n304), .b(new_n303), .c(new_n300), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n297), .o1(new_n306));
  aoi012aa1n06x5               g211(.a(new_n306), .b(new_n294), .c(new_n296), .o1(new_n307));
  nano22aa1n03x5               g212(.a(new_n307), .b(new_n300), .c(new_n304), .out0(new_n308));
  norp02aa1n03x5               g213(.a(new_n305), .b(new_n308), .o1(\s[28] ));
  norb02aa1n03x5               g214(.a(new_n297), .b(new_n304), .out0(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n302), .c(new_n218), .d(new_n293), .o1(new_n311));
  oao003aa1n02x5               g216(.a(\a[28] ), .b(\b[27] ), .c(new_n300), .carry(new_n312));
  xnrc02aa1n02x5               g217(.a(\b[28] ), .b(\a[29] ), .out0(new_n313));
  tech160nm_fiaoi012aa1n02p5x5 g218(.a(new_n313), .b(new_n311), .c(new_n312), .o1(new_n314));
  inv000aa1d42x5               g219(.a(new_n310), .o1(new_n315));
  aoi012aa1n03x5               g220(.a(new_n315), .b(new_n294), .c(new_n296), .o1(new_n316));
  nano22aa1n02x4               g221(.a(new_n316), .b(new_n312), .c(new_n313), .out0(new_n317));
  norp02aa1n03x5               g222(.a(new_n314), .b(new_n317), .o1(\s[29] ));
  xorb03aa1n02x5               g223(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n09x5               g224(.a(new_n297), .b(new_n313), .c(new_n304), .out0(new_n320));
  aoai13aa1n03x5               g225(.a(new_n320), .b(new_n302), .c(new_n218), .d(new_n293), .o1(new_n321));
  oao003aa1n02x5               g226(.a(\a[29] ), .b(\b[28] ), .c(new_n312), .carry(new_n322));
  xnrc02aa1n02x5               g227(.a(\b[29] ), .b(\a[30] ), .out0(new_n323));
  tech160nm_fiaoi012aa1n02p5x5 g228(.a(new_n323), .b(new_n321), .c(new_n322), .o1(new_n324));
  inv000aa1d42x5               g229(.a(new_n320), .o1(new_n325));
  aoi012aa1n03x5               g230(.a(new_n325), .b(new_n294), .c(new_n296), .o1(new_n326));
  nano22aa1n03x5               g231(.a(new_n326), .b(new_n322), .c(new_n323), .out0(new_n327));
  norp02aa1n03x5               g232(.a(new_n324), .b(new_n327), .o1(\s[30] ));
  xnrc02aa1n02x5               g233(.a(\b[30] ), .b(\a[31] ), .out0(new_n329));
  nona32aa1n02x4               g234(.a(new_n297), .b(new_n323), .c(new_n313), .d(new_n304), .out0(new_n330));
  inv000aa1d42x5               g235(.a(new_n330), .o1(new_n331));
  aoai13aa1n03x5               g236(.a(new_n331), .b(new_n302), .c(new_n218), .d(new_n293), .o1(new_n332));
  oao003aa1n02x5               g237(.a(\a[30] ), .b(\b[29] ), .c(new_n322), .carry(new_n333));
  tech160nm_fiaoi012aa1n02p5x5 g238(.a(new_n329), .b(new_n332), .c(new_n333), .o1(new_n334));
  aoi012aa1n06x5               g239(.a(new_n330), .b(new_n294), .c(new_n296), .o1(new_n335));
  nano22aa1n03x5               g240(.a(new_n335), .b(new_n329), .c(new_n333), .out0(new_n336));
  norp02aa1n03x5               g241(.a(new_n334), .b(new_n336), .o1(\s[31] ));
  xnbna2aa1n03x5               g242(.a(new_n101), .b(new_n104), .c(new_n147), .out0(\s[3] ));
  oaoi03aa1n02x5               g243(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n339));
  xorb03aa1n02x5               g244(.a(new_n339), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g245(.a(new_n185), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g246(.a(new_n121), .b(new_n122), .c(new_n185), .o1(new_n342));
  xnrb03aa1n02x5               g247(.a(new_n342), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  orn002aa1n02x5               g248(.a(new_n112), .b(new_n113), .o(new_n344));
  aoai13aa1n02x5               g249(.a(new_n123), .b(new_n344), .c(new_n106), .d(new_n109), .o1(new_n345));
  xorb03aa1n02x5               g250(.a(new_n345), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g251(.a(new_n116), .b(new_n345), .c(new_n117), .o1(new_n347));
  xnrb03aa1n02x5               g252(.a(new_n347), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g253(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


