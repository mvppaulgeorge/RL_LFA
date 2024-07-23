// Benchmark "adder" written by ABC on Wed Jul 17 22:10:34 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n317,
    new_n320, new_n322, new_n323, new_n325;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor022aa1n16x5               g001(.a(\b[7] ), .b(\a[8] ), .o1(new_n97));
  norp02aa1n12x5               g002(.a(\b[6] ), .b(\a[7] ), .o1(new_n98));
  nand22aa1n09x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  ao0012aa1n03x7               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o(new_n100));
  inv000aa1d42x5               g005(.a(\a[6] ), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\b[5] ), .o1(new_n102));
  nor042aa1n04x5               g007(.a(\b[4] ), .b(\a[5] ), .o1(new_n103));
  oaoi03aa1n12x5               g008(.a(new_n101), .b(new_n102), .c(new_n103), .o1(new_n104));
  nanp02aa1n06x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  nona23aa1d18x5               g010(.a(new_n99), .b(new_n105), .c(new_n98), .d(new_n97), .out0(new_n106));
  oabi12aa1n18x5               g011(.a(new_n100), .b(new_n106), .c(new_n104), .out0(new_n107));
  inv040aa1d32x5               g012(.a(\a[4] ), .o1(new_n108));
  inv000aa1d42x5               g013(.a(\b[3] ), .o1(new_n109));
  nand42aa1n02x5               g014(.a(new_n109), .b(new_n108), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(new_n110), .b(new_n111), .o1(new_n112));
  nor022aa1n03x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  oaoi03aa1n03x5               g018(.a(new_n108), .b(new_n109), .c(new_n113), .o1(new_n114));
  nor042aa1n03x5               g019(.a(\b[1] ), .b(\a[2] ), .o1(new_n115));
  nand22aa1n04x5               g020(.a(\b[0] ), .b(\a[1] ), .o1(new_n116));
  nand02aa1d04x5               g021(.a(\b[1] ), .b(\a[2] ), .o1(new_n117));
  tech160nm_fiaoi012aa1n04x5   g022(.a(new_n115), .b(new_n116), .c(new_n117), .o1(new_n118));
  nand42aa1n03x5               g023(.a(\b[2] ), .b(\a[3] ), .o1(new_n119));
  nanb02aa1n06x5               g024(.a(new_n113), .b(new_n119), .out0(new_n120));
  oai013aa1n09x5               g025(.a(new_n114), .b(new_n118), .c(new_n120), .d(new_n112), .o1(new_n121));
  xnrc02aa1n02x5               g026(.a(\b[5] ), .b(\a[6] ), .out0(new_n122));
  xnrc02aa1n02x5               g027(.a(\b[4] ), .b(\a[5] ), .out0(new_n123));
  nor043aa1n06x5               g028(.a(new_n106), .b(new_n123), .c(new_n122), .o1(new_n124));
  tech160nm_fiaoi012aa1n05x5   g029(.a(new_n107), .b(new_n121), .c(new_n124), .o1(new_n125));
  oaoi03aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .c(new_n125), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  inv000aa1d42x5               g032(.a(new_n107), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(new_n121), .b(new_n124), .o1(new_n129));
  nor042aa1n02x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  nor002aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nand42aa1n03x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  tech160nm_fiaoi012aa1n05x5   g037(.a(new_n131), .b(new_n130), .c(new_n132), .o1(new_n133));
  nand42aa1n03x5               g038(.a(\b[8] ), .b(\a[9] ), .o1(new_n134));
  nona23aa1n02x4               g039(.a(new_n132), .b(new_n134), .c(new_n130), .d(new_n131), .out0(new_n135));
  aoai13aa1n02x5               g040(.a(new_n133), .b(new_n135), .c(new_n129), .d(new_n128), .o1(new_n136));
  xorb03aa1n02x5               g041(.a(new_n136), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  inv000aa1d42x5               g042(.a(\a[12] ), .o1(new_n138));
  nor002aa1n12x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  tech160nm_finand02aa1n03p5x5 g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  aoi012aa1n02x5               g045(.a(new_n139), .b(new_n136), .c(new_n140), .o1(new_n141));
  xorb03aa1n02x5               g046(.a(new_n141), .b(\b[11] ), .c(new_n138), .out0(\s[12] ));
  nor022aa1n06x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  tech160nm_finand02aa1n03p5x5 g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  aoi012aa1n02x5               g049(.a(new_n143), .b(new_n139), .c(new_n144), .o1(new_n145));
  nona23aa1n03x5               g050(.a(new_n144), .b(new_n140), .c(new_n139), .d(new_n143), .out0(new_n146));
  oai012aa1n06x5               g051(.a(new_n145), .b(new_n146), .c(new_n133), .o1(new_n147));
  nano23aa1n03x5               g052(.a(new_n130), .b(new_n131), .c(new_n132), .d(new_n134), .out0(new_n148));
  nano23aa1n03x5               g053(.a(new_n139), .b(new_n143), .c(new_n144), .d(new_n140), .out0(new_n149));
  nanp02aa1n02x5               g054(.a(new_n149), .b(new_n148), .o1(new_n150));
  oabi12aa1n02x5               g055(.a(new_n147), .b(new_n125), .c(new_n150), .out0(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1n04x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nand42aa1n06x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  norp02aa1n04x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nand02aa1n03x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n156), .b(new_n155), .out0(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  aoai13aa1n02x5               g063(.a(new_n158), .b(new_n153), .c(new_n151), .d(new_n154), .o1(new_n159));
  aoi112aa1n03x5               g064(.a(new_n153), .b(new_n158), .c(new_n151), .d(new_n154), .o1(new_n160));
  nanb02aa1n02x5               g065(.a(new_n160), .b(new_n159), .out0(\s[14] ));
  nanp02aa1n02x5               g066(.a(new_n129), .b(new_n128), .o1(new_n162));
  aoi012aa1n03x5               g067(.a(new_n155), .b(new_n153), .c(new_n156), .o1(new_n163));
  nano23aa1n06x5               g068(.a(new_n153), .b(new_n155), .c(new_n156), .d(new_n154), .out0(new_n164));
  aob012aa1n02x5               g069(.a(new_n163), .b(new_n147), .c(new_n164), .out0(new_n165));
  nona23aa1n02x4               g070(.a(new_n156), .b(new_n154), .c(new_n153), .d(new_n155), .out0(new_n166));
  norp03aa1n02x5               g071(.a(new_n166), .b(new_n146), .c(new_n135), .o1(new_n167));
  aoi012aa1n02x5               g072(.a(new_n165), .b(new_n162), .c(new_n167), .o1(new_n168));
  nor002aa1d32x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  nand42aa1n06x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  xnbna2aa1n03x5               g076(.a(new_n168), .b(new_n171), .c(new_n170), .out0(\s[15] ));
  nanb02aa1d24x5               g077(.a(new_n169), .b(new_n171), .out0(new_n173));
  inv000aa1d42x5               g078(.a(new_n173), .o1(new_n174));
  aoai13aa1n02x5               g079(.a(new_n174), .b(new_n165), .c(new_n162), .d(new_n167), .o1(new_n175));
  nor002aa1n03x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nand42aa1n03x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nanb02aa1n06x5               g082(.a(new_n176), .b(new_n177), .out0(new_n178));
  xobna2aa1n03x5               g083(.a(new_n178), .b(new_n175), .c(new_n170), .out0(\s[16] ));
  aoi012aa1n02x5               g084(.a(new_n176), .b(new_n169), .c(new_n177), .o1(new_n180));
  oai013aa1n03x4               g085(.a(new_n180), .b(new_n163), .c(new_n173), .d(new_n178), .o1(new_n181));
  nor043aa1n02x5               g086(.a(new_n166), .b(new_n173), .c(new_n178), .o1(new_n182));
  aoi012aa1n09x5               g087(.a(new_n181), .b(new_n147), .c(new_n182), .o1(new_n183));
  nano23aa1n03x7               g088(.a(new_n169), .b(new_n176), .c(new_n177), .d(new_n171), .out0(new_n184));
  nano22aa1n03x7               g089(.a(new_n150), .b(new_n164), .c(new_n184), .out0(new_n185));
  aoai13aa1n12x5               g090(.a(new_n185), .b(new_n107), .c(new_n121), .d(new_n124), .o1(new_n186));
  nor042aa1d18x5               g091(.a(\b[16] ), .b(\a[17] ), .o1(new_n187));
  nand42aa1n03x5               g092(.a(\b[16] ), .b(\a[17] ), .o1(new_n188));
  norb02aa1n02x5               g093(.a(new_n188), .b(new_n187), .out0(new_n189));
  xnbna2aa1n03x5               g094(.a(new_n189), .b(new_n186), .c(new_n183), .out0(\s[17] ));
  nand02aa1d06x5               g095(.a(new_n186), .b(new_n183), .o1(new_n191));
  tech160nm_fiaoi012aa1n05x5   g096(.a(new_n187), .b(new_n191), .c(new_n189), .o1(new_n192));
  inv000aa1d42x5               g097(.a(\a[18] ), .o1(new_n193));
  inv000aa1d42x5               g098(.a(\b[17] ), .o1(new_n194));
  nand02aa1n08x5               g099(.a(new_n194), .b(new_n193), .o1(new_n195));
  nand02aa1d28x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  xnbna2aa1n03x5               g101(.a(new_n192), .b(new_n196), .c(new_n195), .out0(\s[18] ));
  aob012aa1d24x5               g102(.a(new_n195), .b(new_n187), .c(new_n196), .out0(new_n198));
  inv000aa1d42x5               g103(.a(new_n198), .o1(new_n199));
  nano32aa1d15x5               g104(.a(new_n187), .b(new_n196), .c(new_n188), .d(new_n195), .out0(new_n200));
  inv000aa1d42x5               g105(.a(new_n200), .o1(new_n201));
  aoai13aa1n06x5               g106(.a(new_n199), .b(new_n201), .c(new_n186), .d(new_n183), .o1(new_n202));
  xorb03aa1n02x5               g107(.a(new_n202), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g108(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n09x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  norb02aa1n09x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  nor042aa1n12x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nand02aa1d12x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  norb02aa1n15x5               g114(.a(new_n209), .b(new_n208), .out0(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  aoai13aa1n02x5               g116(.a(new_n211), .b(new_n205), .c(new_n202), .d(new_n207), .o1(new_n212));
  nand42aa1n02x5               g117(.a(new_n202), .b(new_n207), .o1(new_n213));
  nona22aa1n02x4               g118(.a(new_n213), .b(new_n211), .c(new_n205), .out0(new_n214));
  nanp02aa1n02x5               g119(.a(new_n214), .b(new_n212), .o1(\s[20] ));
  aoi012aa1d24x5               g120(.a(new_n208), .b(new_n205), .c(new_n209), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  nano23aa1n06x5               g122(.a(new_n205), .b(new_n208), .c(new_n209), .d(new_n206), .out0(new_n218));
  aoi012aa1n12x5               g123(.a(new_n217), .b(new_n218), .c(new_n198), .o1(new_n219));
  nand02aa1d06x5               g124(.a(new_n200), .b(new_n218), .o1(new_n220));
  aoai13aa1n06x5               g125(.a(new_n219), .b(new_n220), .c(new_n186), .d(new_n183), .o1(new_n221));
  xorb03aa1n02x5               g126(.a(new_n221), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  nand22aa1n04x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  norb02aa1n02x5               g129(.a(new_n224), .b(new_n223), .out0(new_n225));
  nor002aa1n04x5               g130(.a(\b[21] ), .b(\a[22] ), .o1(new_n226));
  nand02aa1d06x5               g131(.a(\b[21] ), .b(\a[22] ), .o1(new_n227));
  norb02aa1n02x5               g132(.a(new_n227), .b(new_n226), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoai13aa1n02x5               g134(.a(new_n229), .b(new_n223), .c(new_n221), .d(new_n225), .o1(new_n230));
  nanp02aa1n02x5               g135(.a(new_n221), .b(new_n225), .o1(new_n231));
  nona22aa1n02x4               g136(.a(new_n231), .b(new_n229), .c(new_n223), .out0(new_n232));
  nanp02aa1n02x5               g137(.a(new_n232), .b(new_n230), .o1(\s[22] ));
  inv000aa1n02x5               g138(.a(new_n219), .o1(new_n234));
  ao0012aa1n03x7               g139(.a(new_n226), .b(new_n223), .c(new_n227), .o(new_n235));
  nano23aa1n06x5               g140(.a(new_n223), .b(new_n226), .c(new_n227), .d(new_n224), .out0(new_n236));
  aoi012aa1n02x5               g141(.a(new_n235), .b(new_n234), .c(new_n236), .o1(new_n237));
  nano22aa1n02x5               g142(.a(new_n201), .b(new_n218), .c(new_n236), .out0(new_n238));
  inv000aa1n02x5               g143(.a(new_n238), .o1(new_n239));
  aoai13aa1n06x5               g144(.a(new_n237), .b(new_n239), .c(new_n186), .d(new_n183), .o1(new_n240));
  xorb03aa1n02x5               g145(.a(new_n240), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1n06x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  nand02aa1d04x5               g147(.a(\b[22] ), .b(\a[23] ), .o1(new_n243));
  norb02aa1n02x5               g148(.a(new_n243), .b(new_n242), .out0(new_n244));
  nor042aa1n04x5               g149(.a(\b[23] ), .b(\a[24] ), .o1(new_n245));
  nand02aa1d06x5               g150(.a(\b[23] ), .b(\a[24] ), .o1(new_n246));
  norb02aa1n02x5               g151(.a(new_n246), .b(new_n245), .out0(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  aoai13aa1n02x5               g153(.a(new_n248), .b(new_n242), .c(new_n240), .d(new_n244), .o1(new_n249));
  nand42aa1n02x5               g154(.a(new_n240), .b(new_n244), .o1(new_n250));
  nona22aa1n02x4               g155(.a(new_n250), .b(new_n248), .c(new_n242), .out0(new_n251));
  nanp02aa1n03x5               g156(.a(new_n251), .b(new_n249), .o1(\s[24] ));
  nand23aa1d12x5               g157(.a(new_n198), .b(new_n207), .c(new_n210), .o1(new_n253));
  tech160nm_fiao0012aa1n02p5x5 g158(.a(new_n245), .b(new_n242), .c(new_n246), .o(new_n254));
  nano23aa1n09x5               g159(.a(new_n242), .b(new_n245), .c(new_n246), .d(new_n243), .out0(new_n255));
  aoi012aa1n09x5               g160(.a(new_n254), .b(new_n255), .c(new_n235), .o1(new_n256));
  nand02aa1d04x5               g161(.a(new_n255), .b(new_n236), .o1(new_n257));
  aoai13aa1n12x5               g162(.a(new_n256), .b(new_n257), .c(new_n253), .d(new_n216), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  nano32aa1n02x4               g164(.a(new_n201), .b(new_n255), .c(new_n218), .d(new_n236), .out0(new_n260));
  inv000aa1n02x5               g165(.a(new_n260), .o1(new_n261));
  aoai13aa1n06x5               g166(.a(new_n259), .b(new_n261), .c(new_n186), .d(new_n183), .o1(new_n262));
  xorb03aa1n02x5               g167(.a(new_n262), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g168(.a(\b[24] ), .b(\a[25] ), .o1(new_n264));
  tech160nm_fixorc02aa1n03p5x5 g169(.a(\a[25] ), .b(\b[24] ), .out0(new_n265));
  tech160nm_fixnrc02aa1n05x5   g170(.a(\b[25] ), .b(\a[26] ), .out0(new_n266));
  aoai13aa1n03x5               g171(.a(new_n266), .b(new_n264), .c(new_n262), .d(new_n265), .o1(new_n267));
  nanp02aa1n02x5               g172(.a(new_n262), .b(new_n265), .o1(new_n268));
  nona22aa1n02x4               g173(.a(new_n268), .b(new_n266), .c(new_n264), .out0(new_n269));
  nanp02aa1n02x5               g174(.a(new_n269), .b(new_n267), .o1(\s[26] ));
  inv000aa1d42x5               g175(.a(\a[26] ), .o1(new_n271));
  inv000aa1d42x5               g176(.a(\b[25] ), .o1(new_n272));
  oaoi03aa1n12x5               g177(.a(new_n271), .b(new_n272), .c(new_n264), .o1(new_n273));
  inv000aa1d42x5               g178(.a(new_n273), .o1(new_n274));
  norb02aa1n02x5               g179(.a(new_n265), .b(new_n266), .out0(new_n275));
  aoi012aa1n06x5               g180(.a(new_n274), .b(new_n258), .c(new_n275), .o1(new_n276));
  norb03aa1n03x5               g181(.a(new_n275), .b(new_n220), .c(new_n257), .out0(new_n277));
  inv020aa1n03x5               g182(.a(new_n277), .o1(new_n278));
  aoai13aa1n06x5               g183(.a(new_n276), .b(new_n278), .c(new_n186), .d(new_n183), .o1(new_n279));
  xorb03aa1n03x5               g184(.a(new_n279), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nor042aa1n03x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  xorc02aa1n02x5               g186(.a(\a[27] ), .b(\b[26] ), .out0(new_n282));
  xnrc02aa1n02x5               g187(.a(\b[27] ), .b(\a[28] ), .out0(new_n283));
  aoai13aa1n03x5               g188(.a(new_n283), .b(new_n281), .c(new_n279), .d(new_n282), .o1(new_n284));
  nand02aa1n03x5               g189(.a(new_n258), .b(new_n275), .o1(new_n285));
  nanp02aa1n03x5               g190(.a(new_n285), .b(new_n273), .o1(new_n286));
  aoai13aa1n03x5               g191(.a(new_n282), .b(new_n286), .c(new_n191), .d(new_n277), .o1(new_n287));
  nona22aa1n03x5               g192(.a(new_n287), .b(new_n283), .c(new_n281), .out0(new_n288));
  nanp02aa1n03x5               g193(.a(new_n284), .b(new_n288), .o1(\s[28] ));
  inv000aa1d42x5               g194(.a(\a[28] ), .o1(new_n290));
  inv000aa1d42x5               g195(.a(\b[27] ), .o1(new_n291));
  oaoi03aa1n09x5               g196(.a(new_n290), .b(new_n291), .c(new_n281), .o1(new_n292));
  inv000aa1d42x5               g197(.a(new_n292), .o1(new_n293));
  norb02aa1n02x5               g198(.a(new_n282), .b(new_n283), .out0(new_n294));
  aoai13aa1n02x7               g199(.a(new_n294), .b(new_n286), .c(new_n191), .d(new_n277), .o1(new_n295));
  xnrc02aa1n02x5               g200(.a(\b[28] ), .b(\a[29] ), .out0(new_n296));
  nona22aa1n03x5               g201(.a(new_n295), .b(new_n296), .c(new_n293), .out0(new_n297));
  aoai13aa1n03x5               g202(.a(new_n296), .b(new_n293), .c(new_n279), .d(new_n294), .o1(new_n298));
  nanp02aa1n03x5               g203(.a(new_n298), .b(new_n297), .o1(\s[29] ));
  xorb03aa1n02x5               g204(.a(new_n116), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  tech160nm_fioaoi03aa1n03p5x5 g205(.a(\a[29] ), .b(\b[28] ), .c(new_n292), .o1(new_n301));
  norb03aa1n02x5               g206(.a(new_n282), .b(new_n296), .c(new_n283), .out0(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[29] ), .b(\a[30] ), .out0(new_n303));
  aoai13aa1n03x5               g208(.a(new_n303), .b(new_n301), .c(new_n279), .d(new_n302), .o1(new_n304));
  aoai13aa1n03x5               g209(.a(new_n302), .b(new_n286), .c(new_n191), .d(new_n277), .o1(new_n305));
  nona22aa1n03x5               g210(.a(new_n305), .b(new_n303), .c(new_n301), .out0(new_n306));
  nanp02aa1n03x5               g211(.a(new_n304), .b(new_n306), .o1(\s[30] ));
  nanb02aa1n02x5               g212(.a(new_n303), .b(new_n301), .out0(new_n308));
  oai012aa1n02x7               g213(.a(new_n308), .b(\b[29] ), .c(\a[30] ), .o1(new_n309));
  norb02aa1n02x5               g214(.a(new_n302), .b(new_n303), .out0(new_n310));
  aoai13aa1n02x7               g215(.a(new_n310), .b(new_n286), .c(new_n191), .d(new_n277), .o1(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[30] ), .b(\a[31] ), .out0(new_n312));
  nona22aa1n03x5               g217(.a(new_n311), .b(new_n312), .c(new_n309), .out0(new_n313));
  aoai13aa1n03x5               g218(.a(new_n312), .b(new_n309), .c(new_n279), .d(new_n310), .o1(new_n314));
  nanp02aa1n03x5               g219(.a(new_n314), .b(new_n313), .o1(\s[31] ));
  xnrb03aa1n02x5               g220(.a(new_n118), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g221(.a(\a[3] ), .b(\b[2] ), .c(new_n118), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g223(.a(new_n121), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoib12aa1n02x5               g224(.a(new_n103), .b(new_n121), .c(new_n123), .out0(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[5] ), .c(new_n101), .out0(\s[6] ));
  norp02aa1n02x5               g226(.a(new_n123), .b(new_n122), .o1(new_n322));
  aob012aa1n02x5               g227(.a(new_n104), .b(new_n121), .c(new_n322), .out0(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g229(.a(new_n98), .b(new_n323), .c(new_n105), .o1(new_n325));
  xnrb03aa1n02x5               g230(.a(new_n325), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrb03aa1n02x5               g231(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


