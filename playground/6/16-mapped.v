// Benchmark "adder" written by ABC on Wed Jul 17 15:07:08 2024

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
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n320, new_n323, new_n324, new_n325, new_n326,
    new_n328, new_n330;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n20x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\a[8] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\b[7] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(new_n102), .b(new_n99), .o1(new_n103));
  norp02aa1n04x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  oaoi03aa1n02x5               g009(.a(new_n100), .b(new_n101), .c(new_n104), .o1(new_n105));
  nand42aa1n08x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[6] ), .b(\a[7] ), .o1(new_n107));
  nanb03aa1n02x5               g012(.a(new_n104), .b(new_n107), .c(new_n106), .out0(new_n108));
  norp02aa1n04x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  nor022aa1n06x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  norb03aa1n03x5               g015(.a(new_n106), .b(new_n110), .c(new_n109), .out0(new_n111));
  oai013aa1n03x5               g016(.a(new_n105), .b(new_n111), .c(new_n108), .d(new_n103), .o1(new_n112));
  nand42aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  norp02aa1n03x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nor002aa1n02x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  oai012aa1n02x5               g020(.a(new_n113), .b(new_n115), .c(new_n114), .o1(new_n116));
  norp02aa1n02x5               g021(.a(\b[1] ), .b(\a[2] ), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(\b[0] ), .b(\a[1] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[1] ), .b(\a[2] ), .o1(new_n119));
  tech160nm_fiaoi012aa1n04x5   g024(.a(new_n117), .b(new_n118), .c(new_n119), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(\b[2] ), .b(\a[3] ), .o1(new_n121));
  nona23aa1n09x5               g026(.a(new_n113), .b(new_n121), .c(new_n115), .d(new_n114), .out0(new_n122));
  tech160nm_fioai012aa1n05x5   g027(.a(new_n116), .b(new_n122), .c(new_n120), .o1(new_n123));
  oai112aa1n02x5               g028(.a(new_n102), .b(new_n99), .c(\b[6] ), .d(\a[7] ), .o1(new_n124));
  xnrc02aa1n02x5               g029(.a(\b[4] ), .b(\a[5] ), .out0(new_n125));
  aoi012aa1n02x5               g030(.a(new_n110), .b(\a[7] ), .c(\b[6] ), .o1(new_n126));
  nano23aa1n03x7               g031(.a(new_n124), .b(new_n125), .c(new_n126), .d(new_n106), .out0(new_n127));
  and002aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o(new_n128));
  norp02aa1n02x5               g033(.a(new_n128), .b(new_n97), .o1(new_n129));
  aoai13aa1n06x5               g034(.a(new_n129), .b(new_n112), .c(new_n123), .d(new_n127), .o1(new_n130));
  nor042aa1n04x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  and002aa1n12x5               g036(.a(\b[9] ), .b(\a[10] ), .o(new_n132));
  norp02aa1n02x5               g037(.a(new_n132), .b(new_n131), .o1(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n133), .b(new_n130), .c(new_n98), .out0(\s[10] ));
  inv000aa1n02x5               g039(.a(new_n133), .o1(new_n135));
  oabi12aa1n06x5               g040(.a(new_n132), .b(new_n97), .c(new_n131), .out0(new_n136));
  tech160nm_fioai012aa1n05x5   g041(.a(new_n136), .b(new_n130), .c(new_n135), .o1(new_n137));
  xorb03aa1n02x5               g042(.a(new_n137), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  inv000aa1d42x5               g043(.a(\a[11] ), .o1(new_n139));
  inv000aa1d42x5               g044(.a(\b[10] ), .o1(new_n140));
  tech160nm_fioaoi03aa1n03p5x5 g045(.a(new_n139), .b(new_n140), .c(new_n137), .o1(new_n141));
  orn002aa1n02x5               g046(.a(\a[12] ), .b(\b[11] ), .o(new_n142));
  nand42aa1n08x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n141), .b(new_n143), .c(new_n142), .out0(\s[12] ));
  nor043aa1n06x5               g049(.a(new_n132), .b(new_n131), .c(new_n97), .o1(new_n145));
  norp02aa1n04x5               g050(.a(\b[10] ), .b(\a[11] ), .o1(new_n146));
  nand42aa1n04x5               g051(.a(\b[10] ), .b(\a[11] ), .o1(new_n147));
  norp02aa1n12x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  nano22aa1n03x7               g053(.a(new_n148), .b(new_n147), .c(new_n143), .out0(new_n149));
  nona23aa1d18x5               g054(.a(new_n149), .b(new_n145), .c(new_n128), .d(new_n146), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  aoai13aa1n06x5               g056(.a(new_n151), .b(new_n112), .c(new_n123), .d(new_n127), .o1(new_n152));
  aoai13aa1n04x5               g057(.a(new_n143), .b(new_n148), .c(new_n139), .d(new_n140), .o1(new_n153));
  nona23aa1n06x5               g058(.a(new_n143), .b(new_n147), .c(new_n146), .d(new_n148), .out0(new_n154));
  oai012aa1d24x5               g059(.a(new_n153), .b(new_n154), .c(new_n136), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  nor022aa1n08x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nand42aa1d28x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n158), .b(new_n157), .out0(new_n159));
  xnbna2aa1n03x5               g064(.a(new_n159), .b(new_n152), .c(new_n156), .out0(\s[13] ));
  nanp02aa1n02x5               g065(.a(new_n152), .b(new_n156), .o1(new_n161));
  aoi012aa1n02x5               g066(.a(new_n157), .b(new_n161), .c(new_n158), .o1(new_n162));
  xnrb03aa1n02x5               g067(.a(new_n162), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n09x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nand42aa1n16x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  oai012aa1n02x5               g070(.a(new_n165), .b(new_n164), .c(new_n157), .o1(new_n166));
  nano23aa1d15x5               g071(.a(new_n157), .b(new_n164), .c(new_n165), .d(new_n158), .out0(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  aoai13aa1n06x5               g073(.a(new_n166), .b(new_n168), .c(new_n152), .d(new_n156), .o1(new_n169));
  xorb03aa1n02x5               g074(.a(new_n169), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n03x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nand42aa1n06x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nor002aa1n08x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nand42aa1n06x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nanb02aa1n06x5               g079(.a(new_n173), .b(new_n174), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  aoi112aa1n03x5               g081(.a(new_n171), .b(new_n176), .c(new_n169), .d(new_n172), .o1(new_n177));
  aoai13aa1n04x5               g082(.a(new_n176), .b(new_n171), .c(new_n169), .d(new_n172), .o1(new_n178));
  norb02aa1n02x5               g083(.a(new_n178), .b(new_n177), .out0(\s[16] ));
  inv000aa1n02x5               g084(.a(new_n171), .o1(new_n180));
  nanb03aa1d24x5               g085(.a(new_n173), .b(new_n174), .c(new_n172), .out0(new_n181));
  inv000aa1d42x5               g086(.a(new_n181), .o1(new_n182));
  nano32aa1n09x5               g087(.a(new_n150), .b(new_n182), .c(new_n167), .d(new_n180), .out0(new_n183));
  aoai13aa1n09x5               g088(.a(new_n183), .b(new_n112), .c(new_n123), .d(new_n127), .o1(new_n184));
  nona22aa1n02x4               g089(.a(new_n165), .b(new_n164), .c(new_n157), .out0(new_n185));
  nano23aa1n06x5               g090(.a(new_n185), .b(new_n181), .c(new_n180), .d(new_n158), .out0(new_n186));
  oaoi03aa1n02x5               g091(.a(\a[16] ), .b(\b[15] ), .c(new_n180), .o1(new_n187));
  nano23aa1n03x7               g092(.a(new_n166), .b(new_n175), .c(new_n180), .d(new_n172), .out0(new_n188));
  aoi112aa1n09x5               g093(.a(new_n188), .b(new_n187), .c(new_n155), .d(new_n186), .o1(new_n189));
  nanp02aa1n06x5               g094(.a(new_n184), .b(new_n189), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g096(.a(\a[17] ), .o1(new_n192));
  inv000aa1d42x5               g097(.a(\b[16] ), .o1(new_n193));
  nand42aa1n03x5               g098(.a(new_n193), .b(new_n192), .o1(new_n194));
  and002aa1n02x5               g099(.a(\b[16] ), .b(\a[17] ), .o(new_n195));
  aoai13aa1n02x5               g100(.a(new_n194), .b(new_n195), .c(new_n184), .d(new_n189), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor042aa1n03x5               g102(.a(\b[17] ), .b(\a[18] ), .o1(new_n198));
  nand42aa1n03x5               g103(.a(\b[17] ), .b(\a[18] ), .o1(new_n199));
  nano23aa1d15x5               g104(.a(new_n198), .b(new_n195), .c(new_n194), .d(new_n199), .out0(new_n200));
  inv000aa1d42x5               g105(.a(new_n200), .o1(new_n201));
  aoai13aa1n06x5               g106(.a(new_n199), .b(new_n198), .c(new_n192), .d(new_n193), .o1(new_n202));
  aoai13aa1n06x5               g107(.a(new_n202), .b(new_n201), .c(new_n184), .d(new_n189), .o1(new_n203));
  xorb03aa1n02x5               g108(.a(new_n203), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g109(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n08x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  nand02aa1n03x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  nor002aa1n03x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nand42aa1n02x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  nanb02aa1n06x5               g114(.a(new_n208), .b(new_n209), .out0(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  aoi112aa1n02x7               g116(.a(new_n206), .b(new_n211), .c(new_n203), .d(new_n207), .o1(new_n212));
  aoai13aa1n03x5               g117(.a(new_n211), .b(new_n206), .c(new_n203), .d(new_n207), .o1(new_n213));
  norb02aa1n02x7               g118(.a(new_n213), .b(new_n212), .out0(\s[20] ));
  nona23aa1n09x5               g119(.a(new_n200), .b(new_n207), .c(new_n210), .d(new_n206), .out0(new_n215));
  oai012aa1n02x5               g120(.a(new_n209), .b(new_n208), .c(new_n206), .o1(new_n216));
  nona23aa1n06x5               g121(.a(new_n209), .b(new_n207), .c(new_n206), .d(new_n208), .out0(new_n217));
  oai012aa1n12x5               g122(.a(new_n216), .b(new_n217), .c(new_n202), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  aoai13aa1n06x5               g124(.a(new_n219), .b(new_n215), .c(new_n184), .d(new_n189), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n02x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  nanp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  xorc02aa1n02x5               g128(.a(\a[22] ), .b(\b[21] ), .out0(new_n224));
  aoi112aa1n02x7               g129(.a(new_n222), .b(new_n224), .c(new_n220), .d(new_n223), .o1(new_n225));
  aoai13aa1n03x5               g130(.a(new_n224), .b(new_n222), .c(new_n220), .d(new_n223), .o1(new_n226));
  norb02aa1n02x7               g131(.a(new_n226), .b(new_n225), .out0(\s[22] ));
  inv000aa1d42x5               g132(.a(\a[22] ), .o1(new_n228));
  inv000aa1d42x5               g133(.a(\b[21] ), .o1(new_n229));
  nanp02aa1n02x5               g134(.a(new_n229), .b(new_n228), .o1(new_n230));
  nand42aa1n03x5               g135(.a(\b[21] ), .b(\a[22] ), .o1(new_n231));
  oai112aa1n02x5               g136(.a(new_n230), .b(new_n231), .c(\b[20] ), .d(\a[21] ), .o1(new_n232));
  norb02aa1n02x5               g137(.a(new_n223), .b(new_n232), .out0(new_n233));
  nanb02aa1n02x5               g138(.a(new_n215), .b(new_n233), .out0(new_n234));
  aoi022aa1n02x5               g139(.a(new_n218), .b(new_n233), .c(new_n231), .d(new_n232), .o1(new_n235));
  aoai13aa1n06x5               g140(.a(new_n235), .b(new_n234), .c(new_n184), .d(new_n189), .o1(new_n236));
  xorb03aa1n02x5               g141(.a(new_n236), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1n02x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  nanp02aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  orn002aa1n02x5               g144(.a(\a[24] ), .b(\b[23] ), .o(new_n240));
  nanp02aa1n02x5               g145(.a(\b[23] ), .b(\a[24] ), .o1(new_n241));
  aoi122aa1n03x5               g146(.a(new_n238), .b(new_n240), .c(new_n241), .d(new_n236), .e(new_n239), .o1(new_n242));
  inv000aa1n02x5               g147(.a(new_n238), .o1(new_n243));
  norb02aa1n02x5               g148(.a(new_n239), .b(new_n238), .out0(new_n244));
  nanp02aa1n03x5               g149(.a(new_n236), .b(new_n244), .o1(new_n245));
  nanp02aa1n02x5               g150(.a(new_n240), .b(new_n241), .o1(new_n246));
  aoi012aa1n06x5               g151(.a(new_n246), .b(new_n245), .c(new_n243), .o1(new_n247));
  norp02aa1n03x5               g152(.a(new_n247), .b(new_n242), .o1(\s[24] ));
  nanp03aa1n02x5               g153(.a(new_n240), .b(new_n239), .c(new_n241), .o1(new_n249));
  nano23aa1n06x5               g154(.a(new_n249), .b(new_n232), .c(new_n223), .d(new_n243), .out0(new_n250));
  nanb02aa1n02x5               g155(.a(new_n215), .b(new_n250), .out0(new_n251));
  oaoi03aa1n02x5               g156(.a(new_n228), .b(new_n229), .c(new_n222), .o1(new_n252));
  nano23aa1n02x5               g157(.a(new_n252), .b(new_n246), .c(new_n243), .d(new_n239), .out0(new_n253));
  oaoi03aa1n02x5               g158(.a(\a[24] ), .b(\b[23] ), .c(new_n243), .o1(new_n254));
  aoi112aa1n06x5               g159(.a(new_n254), .b(new_n253), .c(new_n218), .d(new_n250), .o1(new_n255));
  aoai13aa1n04x5               g160(.a(new_n255), .b(new_n251), .c(new_n184), .d(new_n189), .o1(new_n256));
  xorb03aa1n02x5               g161(.a(new_n256), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(\b[24] ), .b(\a[25] ), .o1(new_n259));
  tech160nm_fixorc02aa1n03p5x5 g164(.a(\a[26] ), .b(\b[25] ), .out0(new_n260));
  aoi112aa1n02x7               g165(.a(new_n258), .b(new_n260), .c(new_n256), .d(new_n259), .o1(new_n261));
  aoai13aa1n03x5               g166(.a(new_n260), .b(new_n258), .c(new_n256), .d(new_n259), .o1(new_n262));
  norb02aa1n02x7               g167(.a(new_n262), .b(new_n261), .out0(\s[26] ));
  norp03aa1n02x5               g168(.a(new_n111), .b(new_n108), .c(new_n103), .o1(new_n264));
  norb02aa1n02x5               g169(.a(new_n105), .b(new_n264), .out0(new_n265));
  nanp02aa1n02x5               g170(.a(new_n123), .b(new_n127), .o1(new_n266));
  nanp02aa1n02x5               g171(.a(new_n266), .b(new_n265), .o1(new_n267));
  nanp02aa1n02x5               g172(.a(new_n155), .b(new_n186), .o1(new_n268));
  nona22aa1n02x4               g173(.a(new_n268), .b(new_n188), .c(new_n187), .out0(new_n269));
  nano22aa1n03x7               g174(.a(new_n258), .b(new_n260), .c(new_n259), .out0(new_n270));
  nano22aa1n03x7               g175(.a(new_n215), .b(new_n270), .c(new_n250), .out0(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n269), .c(new_n267), .d(new_n183), .o1(new_n272));
  nanp02aa1n02x5               g177(.a(new_n218), .b(new_n250), .o1(new_n273));
  nona22aa1n03x5               g178(.a(new_n273), .b(new_n253), .c(new_n254), .out0(new_n274));
  nanp02aa1n02x5               g179(.a(\b[25] ), .b(\a[26] ), .o1(new_n275));
  inv000aa1d42x5               g180(.a(\a[25] ), .o1(new_n276));
  oaib12aa1n02x5               g181(.a(new_n260), .b(\b[24] ), .c(new_n276), .out0(new_n277));
  aoi022aa1n09x5               g182(.a(new_n274), .b(new_n270), .c(new_n275), .d(new_n277), .o1(new_n278));
  xorc02aa1n02x5               g183(.a(\a[27] ), .b(\b[26] ), .out0(new_n279));
  xnbna2aa1n03x5               g184(.a(new_n279), .b(new_n272), .c(new_n278), .out0(\s[27] ));
  orn002aa1n02x5               g185(.a(\a[27] ), .b(\b[26] ), .o(new_n281));
  aobi12aa1n02x5               g186(.a(new_n279), .b(new_n272), .c(new_n278), .out0(new_n282));
  norp02aa1n02x5               g187(.a(\b[27] ), .b(\a[28] ), .o1(new_n283));
  nanp02aa1n02x5               g188(.a(\b[27] ), .b(\a[28] ), .o1(new_n284));
  nanb02aa1n02x5               g189(.a(new_n283), .b(new_n284), .out0(new_n285));
  nano22aa1n03x5               g190(.a(new_n282), .b(new_n281), .c(new_n285), .out0(new_n286));
  inv000aa1d42x5               g191(.a(new_n270), .o1(new_n287));
  nanp02aa1n02x5               g192(.a(new_n277), .b(new_n275), .o1(new_n288));
  oai012aa1n06x5               g193(.a(new_n288), .b(new_n255), .c(new_n287), .o1(new_n289));
  aoai13aa1n03x5               g194(.a(new_n279), .b(new_n289), .c(new_n190), .d(new_n271), .o1(new_n290));
  tech160nm_fiaoi012aa1n02p5x5 g195(.a(new_n285), .b(new_n290), .c(new_n281), .o1(new_n291));
  norp02aa1n03x5               g196(.a(new_n291), .b(new_n286), .o1(\s[28] ));
  xnrc02aa1n02x5               g197(.a(\b[28] ), .b(\a[29] ), .out0(new_n293));
  nanp02aa1n02x5               g198(.a(\b[26] ), .b(\a[27] ), .o1(new_n294));
  nano22aa1n02x4               g199(.a(new_n285), .b(new_n281), .c(new_n294), .out0(new_n295));
  aoai13aa1n03x5               g200(.a(new_n295), .b(new_n289), .c(new_n190), .d(new_n271), .o1(new_n296));
  oaib12aa1n02x5               g201(.a(new_n284), .b(new_n283), .c(new_n281), .out0(new_n297));
  tech160nm_fiaoi012aa1n02p5x5 g202(.a(new_n293), .b(new_n296), .c(new_n297), .o1(new_n298));
  aobi12aa1n02x7               g203(.a(new_n295), .b(new_n272), .c(new_n278), .out0(new_n299));
  nano22aa1n03x5               g204(.a(new_n299), .b(new_n293), .c(new_n297), .out0(new_n300));
  nor002aa1n02x5               g205(.a(new_n298), .b(new_n300), .o1(\s[29] ));
  xorb03aa1n02x5               g206(.a(new_n118), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  oao003aa1n02x5               g207(.a(\a[29] ), .b(\b[28] ), .c(new_n297), .carry(new_n303));
  nano23aa1n02x4               g208(.a(new_n293), .b(new_n283), .c(new_n279), .d(new_n284), .out0(new_n304));
  aoai13aa1n03x5               g209(.a(new_n304), .b(new_n289), .c(new_n190), .d(new_n271), .o1(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[29] ), .b(\a[30] ), .out0(new_n306));
  aoi012aa1n02x7               g211(.a(new_n306), .b(new_n305), .c(new_n303), .o1(new_n307));
  aobi12aa1n02x5               g212(.a(new_n304), .b(new_n272), .c(new_n278), .out0(new_n308));
  nano22aa1n03x5               g213(.a(new_n308), .b(new_n303), .c(new_n306), .out0(new_n309));
  norp02aa1n03x5               g214(.a(new_n307), .b(new_n309), .o1(\s[30] ));
  norb03aa1n02x5               g215(.a(new_n295), .b(new_n293), .c(new_n306), .out0(new_n311));
  aobi12aa1n03x5               g216(.a(new_n311), .b(new_n272), .c(new_n278), .out0(new_n312));
  oao003aa1n02x5               g217(.a(\a[30] ), .b(\b[29] ), .c(new_n303), .carry(new_n313));
  xnrc02aa1n02x5               g218(.a(\b[30] ), .b(\a[31] ), .out0(new_n314));
  nano22aa1n03x5               g219(.a(new_n312), .b(new_n313), .c(new_n314), .out0(new_n315));
  aoai13aa1n03x5               g220(.a(new_n311), .b(new_n289), .c(new_n190), .d(new_n271), .o1(new_n316));
  tech160nm_fiaoi012aa1n02p5x5 g221(.a(new_n314), .b(new_n316), .c(new_n313), .o1(new_n317));
  nor002aa1n02x5               g222(.a(new_n317), .b(new_n315), .o1(\s[31] ));
  xnrb03aa1n02x5               g223(.a(new_n120), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g224(.a(\a[3] ), .b(\b[2] ), .c(new_n120), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g226(.a(new_n123), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  norb02aa1n02x5               g227(.a(new_n106), .b(new_n110), .out0(new_n323));
  oaoi13aa1n02x5               g228(.a(new_n125), .b(new_n116), .c(new_n122), .d(new_n120), .o1(new_n324));
  oab012aa1n02x4               g229(.a(new_n323), .b(new_n324), .c(new_n109), .out0(new_n325));
  nona22aa1n02x4               g230(.a(new_n323), .b(new_n324), .c(new_n109), .out0(new_n326));
  nanb02aa1n02x5               g231(.a(new_n325), .b(new_n326), .out0(\s[6] ));
  oai013aa1n02x4               g232(.a(new_n106), .b(new_n324), .c(new_n109), .d(new_n110), .o1(new_n328));
  xnrb03aa1n02x5               g233(.a(new_n328), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g234(.a(\a[7] ), .b(\b[6] ), .c(new_n328), .o1(new_n330));
  xnrc02aa1n02x5               g235(.a(new_n330), .b(new_n103), .out0(\s[8] ));
  xnbna2aa1n03x5               g236(.a(new_n129), .b(new_n266), .c(new_n265), .out0(\s[9] ));
endmodule

