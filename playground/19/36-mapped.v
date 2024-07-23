// Benchmark "adder" written by ABC on Wed Jul 17 22:01:20 2024

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
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n168, new_n169, new_n170, new_n171, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n316, new_n318, new_n320, new_n321,
    new_n322;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n03x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand02aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  nanb02aa1n03x5               g003(.a(new_n97), .b(new_n98), .out0(new_n99));
  nor042aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  xorc02aa1n02x5               g005(.a(\a[9] ), .b(\b[8] ), .out0(new_n101));
  nanp02aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  norp02aa1n02x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  tech160nm_fioai012aa1n04x5   g009(.a(new_n102), .b(new_n104), .c(new_n103), .o1(new_n105));
  nor002aa1n02x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[3] ), .b(\a[4] ), .o1(new_n107));
  nor022aa1n04x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[2] ), .b(\a[3] ), .o1(new_n109));
  nona23aa1n06x5               g014(.a(new_n109), .b(new_n107), .c(new_n106), .d(new_n108), .out0(new_n110));
  aoi012aa1n02x5               g015(.a(new_n106), .b(new_n108), .c(new_n107), .o1(new_n111));
  oai012aa1n06x5               g016(.a(new_n111), .b(new_n110), .c(new_n105), .o1(new_n112));
  norp02aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nor002aa1n03x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nona23aa1n02x4               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[5] ), .b(\a[6] ), .out0(new_n118));
  xnrc02aa1n12x5               g023(.a(\b[7] ), .b(\a[8] ), .out0(new_n119));
  nor043aa1n03x5               g024(.a(new_n117), .b(new_n118), .c(new_n119), .o1(new_n120));
  nand42aa1n02x5               g025(.a(new_n112), .b(new_n120), .o1(new_n121));
  inv000aa1n02x5               g026(.a(new_n113), .o1(new_n122));
  orn002aa1n02x5               g027(.a(\a[8] ), .b(\b[7] ), .o(new_n123));
  inv000aa1d42x5               g028(.a(\a[6] ), .o1(new_n124));
  inv000aa1d42x5               g029(.a(\b[5] ), .o1(new_n125));
  oaoi03aa1n02x5               g030(.a(new_n124), .b(new_n125), .c(new_n115), .o1(new_n126));
  aob012aa1n02x5               g031(.a(new_n114), .b(\b[7] ), .c(\a[8] ), .out0(new_n127));
  aoai13aa1n06x5               g032(.a(new_n123), .b(new_n127), .c(new_n126), .d(new_n122), .o1(new_n128));
  nanb02aa1n06x5               g033(.a(new_n128), .b(new_n121), .out0(new_n129));
  aoai13aa1n02x5               g034(.a(new_n99), .b(new_n100), .c(new_n129), .d(new_n101), .o1(new_n130));
  inv000aa1d42x5               g035(.a(new_n98), .o1(new_n131));
  aoai13aa1n02x5               g036(.a(new_n101), .b(new_n128), .c(new_n112), .d(new_n120), .o1(new_n132));
  nona32aa1n02x4               g037(.a(new_n132), .b(new_n100), .c(new_n131), .d(new_n97), .out0(new_n133));
  nanp02aa1n02x5               g038(.a(new_n130), .b(new_n133), .o1(\s[10] ));
  nand42aa1n06x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  orn002aa1n02x7               g040(.a(\a[11] ), .b(\b[10] ), .o(new_n136));
  nanp02aa1n02x5               g041(.a(new_n136), .b(new_n135), .o1(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n137), .b(new_n133), .c(new_n98), .out0(\s[11] ));
  nona22aa1n02x4               g043(.a(new_n133), .b(new_n137), .c(new_n131), .out0(new_n139));
  norp02aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand42aa1n10x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n139), .c(new_n136), .out0(\s[12] ));
  nano22aa1n03x5               g048(.a(new_n140), .b(new_n135), .c(new_n141), .out0(new_n144));
  nano32aa1n02x4               g049(.a(new_n99), .b(new_n144), .c(new_n101), .d(new_n136), .out0(new_n145));
  aoai13aa1n02x5               g050(.a(new_n145), .b(new_n128), .c(new_n112), .d(new_n120), .o1(new_n146));
  oai012aa1n02x5               g051(.a(new_n98), .b(\b[10] ), .c(\a[11] ), .o1(new_n147));
  oab012aa1n03x5               g052(.a(new_n147), .b(new_n97), .c(new_n100), .out0(new_n148));
  oaoi03aa1n03x5               g053(.a(\a[12] ), .b(\b[11] ), .c(new_n136), .o1(new_n149));
  aoi012aa1n02x5               g054(.a(new_n149), .b(new_n148), .c(new_n144), .o1(new_n150));
  nanp02aa1n03x5               g055(.a(new_n146), .b(new_n150), .o1(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n04x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanp02aa1n04x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  aoi012aa1n02x5               g059(.a(new_n153), .b(new_n151), .c(new_n154), .o1(new_n155));
  xnrb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n04x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nand02aa1n08x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nano23aa1d12x5               g063(.a(new_n153), .b(new_n157), .c(new_n158), .d(new_n154), .out0(new_n159));
  aoi012aa1d18x5               g064(.a(new_n157), .b(new_n153), .c(new_n158), .o1(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  nor042aa1n12x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  nand42aa1n08x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n163), .b(new_n162), .out0(new_n164));
  aoai13aa1n06x5               g069(.a(new_n164), .b(new_n161), .c(new_n151), .d(new_n159), .o1(new_n165));
  aoi112aa1n02x5               g070(.a(new_n164), .b(new_n161), .c(new_n151), .d(new_n159), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n165), .b(new_n166), .out0(\s[15] ));
  inv000aa1d42x5               g072(.a(new_n162), .o1(new_n168));
  nor042aa1n04x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nand42aa1n08x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  xnbna2aa1n03x5               g076(.a(new_n171), .b(new_n165), .c(new_n168), .out0(\s[16] ));
  nanb03aa1n03x5               g077(.a(new_n99), .b(new_n101), .c(new_n136), .out0(new_n173));
  nano23aa1d15x5               g078(.a(new_n162), .b(new_n169), .c(new_n170), .d(new_n163), .out0(new_n174));
  nano32aa1n03x7               g079(.a(new_n173), .b(new_n174), .c(new_n144), .d(new_n159), .out0(new_n175));
  aoai13aa1n06x5               g080(.a(new_n175), .b(new_n128), .c(new_n112), .d(new_n120), .o1(new_n176));
  aoai13aa1n03x5               g081(.a(new_n159), .b(new_n149), .c(new_n148), .d(new_n144), .o1(new_n177));
  nanp02aa1n03x5               g082(.a(new_n177), .b(new_n160), .o1(new_n178));
  nand42aa1n04x5               g083(.a(new_n178), .b(new_n174), .o1(new_n179));
  oai012aa1n02x5               g084(.a(new_n170), .b(new_n169), .c(new_n162), .o1(new_n180));
  nanp03aa1d12x5               g085(.a(new_n176), .b(new_n179), .c(new_n180), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g087(.a(\a[18] ), .o1(new_n183));
  inv040aa1d28x5               g088(.a(\a[17] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\b[16] ), .o1(new_n185));
  oaoi03aa1n03x5               g090(.a(new_n184), .b(new_n185), .c(new_n181), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[17] ), .c(new_n183), .out0(\s[18] ));
  inv000aa1d42x5               g092(.a(new_n174), .o1(new_n188));
  aoai13aa1n06x5               g093(.a(new_n180), .b(new_n188), .c(new_n177), .d(new_n160), .o1(new_n189));
  xroi22aa1d06x4               g094(.a(new_n184), .b(\b[16] ), .c(new_n183), .d(\b[17] ), .out0(new_n190));
  aoai13aa1n02x5               g095(.a(new_n190), .b(new_n189), .c(new_n129), .d(new_n175), .o1(new_n191));
  norp02aa1n02x5               g096(.a(\b[17] ), .b(\a[18] ), .o1(new_n192));
  nand22aa1n04x5               g097(.a(\b[17] ), .b(\a[18] ), .o1(new_n193));
  aoi013aa1n09x5               g098(.a(new_n192), .b(new_n193), .c(new_n184), .d(new_n185), .o1(new_n194));
  nor002aa1d32x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nand22aa1n02x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  nanb02aa1n02x5               g101(.a(new_n195), .b(new_n196), .out0(new_n197));
  inv000aa1d42x5               g102(.a(new_n197), .o1(new_n198));
  xnbna2aa1n03x5               g103(.a(new_n198), .b(new_n191), .c(new_n194), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1n06x5               g105(.a(new_n195), .o1(new_n201));
  nona22aa1n02x4               g106(.a(new_n193), .b(\b[16] ), .c(\a[17] ), .out0(new_n202));
  oaib12aa1n02x5               g107(.a(new_n202), .b(\b[17] ), .c(new_n183), .out0(new_n203));
  aoai13aa1n06x5               g108(.a(new_n198), .b(new_n203), .c(new_n181), .d(new_n190), .o1(new_n204));
  nor022aa1n06x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nand02aa1n02x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanb02aa1n02x5               g111(.a(new_n205), .b(new_n206), .out0(new_n207));
  tech160nm_fiaoi012aa1n03p5x5 g112(.a(new_n207), .b(new_n204), .c(new_n201), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n207), .o1(new_n209));
  nona22aa1n02x5               g114(.a(new_n204), .b(new_n209), .c(new_n195), .out0(new_n210));
  norb02aa1n03x4               g115(.a(new_n210), .b(new_n208), .out0(\s[20] ));
  nano23aa1n06x5               g116(.a(new_n195), .b(new_n205), .c(new_n206), .d(new_n196), .out0(new_n212));
  nand22aa1n03x5               g117(.a(new_n190), .b(new_n212), .o1(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  aoai13aa1n06x5               g119(.a(new_n214), .b(new_n189), .c(new_n129), .d(new_n175), .o1(new_n215));
  nona23aa1n06x5               g120(.a(new_n206), .b(new_n196), .c(new_n195), .d(new_n205), .out0(new_n216));
  oaoi03aa1n09x5               g121(.a(\a[20] ), .b(\b[19] ), .c(new_n201), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  oai012aa1n18x5               g123(.a(new_n218), .b(new_n216), .c(new_n194), .o1(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  nor002aa1d32x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  nanp02aa1n02x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  xnbna2aa1n03x5               g128(.a(new_n223), .b(new_n215), .c(new_n220), .out0(\s[21] ));
  inv000aa1d42x5               g129(.a(new_n221), .o1(new_n225));
  aoai13aa1n04x5               g130(.a(new_n223), .b(new_n219), .c(new_n181), .d(new_n214), .o1(new_n226));
  xnrc02aa1n02x5               g131(.a(\b[21] ), .b(\a[22] ), .out0(new_n227));
  tech160nm_fiaoi012aa1n03p5x5 g132(.a(new_n227), .b(new_n226), .c(new_n225), .o1(new_n228));
  aobi12aa1n02x7               g133(.a(new_n223), .b(new_n215), .c(new_n220), .out0(new_n229));
  nano22aa1n03x7               g134(.a(new_n229), .b(new_n225), .c(new_n227), .out0(new_n230));
  nor002aa1n02x5               g135(.a(new_n228), .b(new_n230), .o1(\s[22] ));
  nano22aa1n03x7               g136(.a(new_n227), .b(new_n225), .c(new_n222), .out0(new_n232));
  and003aa1n02x5               g137(.a(new_n190), .b(new_n232), .c(new_n212), .o(new_n233));
  aoai13aa1n06x5               g138(.a(new_n233), .b(new_n189), .c(new_n129), .d(new_n175), .o1(new_n234));
  aoai13aa1n06x5               g139(.a(new_n232), .b(new_n217), .c(new_n212), .d(new_n203), .o1(new_n235));
  oao003aa1n12x5               g140(.a(\a[22] ), .b(\b[21] ), .c(new_n225), .carry(new_n236));
  nanp02aa1n02x5               g141(.a(new_n235), .b(new_n236), .o1(new_n237));
  xnrc02aa1n12x5               g142(.a(\b[22] ), .b(\a[23] ), .out0(new_n238));
  aoib12aa1n03x5               g143(.a(new_n238), .b(new_n234), .c(new_n237), .out0(new_n239));
  inv000aa1d42x5               g144(.a(new_n238), .o1(new_n240));
  aoi112aa1n02x5               g145(.a(new_n240), .b(new_n237), .c(new_n181), .d(new_n233), .o1(new_n241));
  norp02aa1n02x5               g146(.a(new_n239), .b(new_n241), .o1(\s[23] ));
  nor042aa1n03x5               g147(.a(\b[22] ), .b(\a[23] ), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  aoai13aa1n04x5               g149(.a(new_n240), .b(new_n237), .c(new_n181), .d(new_n233), .o1(new_n245));
  xnrc02aa1n02x5               g150(.a(\b[23] ), .b(\a[24] ), .out0(new_n246));
  tech160nm_fiaoi012aa1n03p5x5 g151(.a(new_n246), .b(new_n245), .c(new_n244), .o1(new_n247));
  nano22aa1n03x7               g152(.a(new_n239), .b(new_n244), .c(new_n246), .out0(new_n248));
  norp02aa1n02x5               g153(.a(new_n247), .b(new_n248), .o1(\s[24] ));
  nor042aa1n02x5               g154(.a(new_n246), .b(new_n238), .o1(new_n250));
  nano22aa1n03x7               g155(.a(new_n213), .b(new_n232), .c(new_n250), .out0(new_n251));
  aoai13aa1n02x5               g156(.a(new_n251), .b(new_n189), .c(new_n129), .d(new_n175), .o1(new_n252));
  inv000aa1n02x5               g157(.a(new_n250), .o1(new_n253));
  oao003aa1n02x5               g158(.a(\a[24] ), .b(\b[23] ), .c(new_n244), .carry(new_n254));
  aoai13aa1n06x5               g159(.a(new_n254), .b(new_n253), .c(new_n235), .d(new_n236), .o1(new_n255));
  xnrc02aa1n12x5               g160(.a(\b[24] ), .b(\a[25] ), .out0(new_n256));
  aoib12aa1n06x5               g161(.a(new_n256), .b(new_n252), .c(new_n255), .out0(new_n257));
  inv000aa1d42x5               g162(.a(new_n256), .o1(new_n258));
  aoi112aa1n02x5               g163(.a(new_n258), .b(new_n255), .c(new_n181), .d(new_n251), .o1(new_n259));
  norp02aa1n02x5               g164(.a(new_n257), .b(new_n259), .o1(\s[25] ));
  nor042aa1n03x5               g165(.a(\b[24] ), .b(\a[25] ), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  aoai13aa1n03x5               g167(.a(new_n258), .b(new_n255), .c(new_n181), .d(new_n251), .o1(new_n263));
  xnrc02aa1n02x5               g168(.a(\b[25] ), .b(\a[26] ), .out0(new_n264));
  tech160nm_fiaoi012aa1n05x5   g169(.a(new_n264), .b(new_n263), .c(new_n262), .o1(new_n265));
  nano22aa1n03x7               g170(.a(new_n257), .b(new_n262), .c(new_n264), .out0(new_n266));
  norp02aa1n02x5               g171(.a(new_n265), .b(new_n266), .o1(\s[26] ));
  nor042aa1n04x5               g172(.a(new_n264), .b(new_n256), .o1(new_n268));
  nano32aa1n03x7               g173(.a(new_n213), .b(new_n268), .c(new_n232), .d(new_n250), .out0(new_n269));
  aoai13aa1n06x5               g174(.a(new_n269), .b(new_n189), .c(new_n129), .d(new_n175), .o1(new_n270));
  oao003aa1n02x5               g175(.a(\a[26] ), .b(\b[25] ), .c(new_n262), .carry(new_n271));
  aobi12aa1n06x5               g176(.a(new_n271), .b(new_n255), .c(new_n268), .out0(new_n272));
  xorc02aa1n02x5               g177(.a(\a[27] ), .b(\b[26] ), .out0(new_n273));
  xnbna2aa1n03x5               g178(.a(new_n273), .b(new_n272), .c(new_n270), .out0(\s[27] ));
  norp02aa1n02x5               g179(.a(\b[26] ), .b(\a[27] ), .o1(new_n275));
  inv040aa1n03x5               g180(.a(new_n275), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n236), .o1(new_n277));
  aoai13aa1n06x5               g182(.a(new_n250), .b(new_n277), .c(new_n219), .d(new_n232), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n268), .o1(new_n279));
  aoai13aa1n06x5               g184(.a(new_n271), .b(new_n279), .c(new_n278), .d(new_n254), .o1(new_n280));
  aoai13aa1n03x5               g185(.a(new_n273), .b(new_n280), .c(new_n181), .d(new_n269), .o1(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[27] ), .b(\a[28] ), .out0(new_n282));
  tech160nm_fiaoi012aa1n02p5x5 g187(.a(new_n282), .b(new_n281), .c(new_n276), .o1(new_n283));
  aobi12aa1n03x5               g188(.a(new_n273), .b(new_n272), .c(new_n270), .out0(new_n284));
  nano22aa1n03x5               g189(.a(new_n284), .b(new_n276), .c(new_n282), .out0(new_n285));
  norp02aa1n03x5               g190(.a(new_n283), .b(new_n285), .o1(\s[28] ));
  xnrc02aa1n02x5               g191(.a(\b[28] ), .b(\a[29] ), .out0(new_n287));
  norb02aa1n02x5               g192(.a(new_n273), .b(new_n282), .out0(new_n288));
  aoai13aa1n02x5               g193(.a(new_n288), .b(new_n280), .c(new_n181), .d(new_n269), .o1(new_n289));
  oao003aa1n02x5               g194(.a(\a[28] ), .b(\b[27] ), .c(new_n276), .carry(new_n290));
  tech160nm_fiaoi012aa1n02p5x5 g195(.a(new_n287), .b(new_n289), .c(new_n290), .o1(new_n291));
  aobi12aa1n06x5               g196(.a(new_n288), .b(new_n272), .c(new_n270), .out0(new_n292));
  nano22aa1n03x5               g197(.a(new_n292), .b(new_n287), .c(new_n290), .out0(new_n293));
  norp02aa1n03x5               g198(.a(new_n291), .b(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g199(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g200(.a(\b[29] ), .b(\a[30] ), .out0(new_n296));
  norb03aa1n02x5               g201(.a(new_n273), .b(new_n287), .c(new_n282), .out0(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n280), .c(new_n181), .d(new_n269), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n290), .carry(new_n299));
  tech160nm_fiaoi012aa1n03p5x5 g204(.a(new_n296), .b(new_n298), .c(new_n299), .o1(new_n300));
  aobi12aa1n03x5               g205(.a(new_n297), .b(new_n272), .c(new_n270), .out0(new_n301));
  nano22aa1n03x5               g206(.a(new_n301), .b(new_n296), .c(new_n299), .out0(new_n302));
  norp02aa1n03x5               g207(.a(new_n300), .b(new_n302), .o1(\s[30] ));
  norb02aa1n02x5               g208(.a(new_n297), .b(new_n296), .out0(new_n304));
  aoai13aa1n02x5               g209(.a(new_n304), .b(new_n280), .c(new_n181), .d(new_n269), .o1(new_n305));
  oao003aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .c(new_n299), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[30] ), .b(\a[31] ), .out0(new_n307));
  tech160nm_fiaoi012aa1n02p5x5 g212(.a(new_n307), .b(new_n305), .c(new_n306), .o1(new_n308));
  aobi12aa1n03x5               g213(.a(new_n304), .b(new_n272), .c(new_n270), .out0(new_n309));
  nano22aa1n03x5               g214(.a(new_n309), .b(new_n306), .c(new_n307), .out0(new_n310));
  norp02aa1n03x5               g215(.a(new_n308), .b(new_n310), .o1(\s[31] ));
  xnrb03aa1n02x5               g216(.a(new_n105), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g217(.a(\a[3] ), .b(\b[2] ), .c(new_n105), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g219(.a(new_n112), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  tech160nm_fiao0012aa1n02p5x5 g220(.a(new_n115), .b(new_n112), .c(new_n116), .o(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oao003aa1n02x5               g222(.a(new_n124), .b(new_n125), .c(new_n316), .carry(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  inv000aa1d42x5               g224(.a(new_n119), .o1(new_n320));
  aoai13aa1n02x5               g225(.a(new_n320), .b(new_n113), .c(new_n318), .d(new_n114), .o1(new_n321));
  aoi112aa1n02x5               g226(.a(new_n320), .b(new_n113), .c(new_n318), .d(new_n114), .o1(new_n322));
  norb02aa1n02x5               g227(.a(new_n321), .b(new_n322), .out0(\s[8] ));
  xorb03aa1n02x5               g228(.a(new_n129), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


