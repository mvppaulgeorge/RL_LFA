// Benchmark "adder" written by ABC on Thu Jul 18 03:21:48 2024

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
    new_n134, new_n135, new_n136, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n161, new_n162, new_n163, new_n164, new_n165,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n178, new_n180, new_n181, new_n182,
    new_n183, new_n184, new_n185, new_n188, new_n189, new_n190, new_n191,
    new_n192, new_n193, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n212, new_n213, new_n214,
    new_n215, new_n216, new_n217, new_n218, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n231, new_n232, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n303,
    new_n305, new_n307, new_n309, new_n311;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nor042aa1n12x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  and002aa1n12x5               g003(.a(\b[1] ), .b(\a[2] ), .o(new_n99));
  nand22aa1n12x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nor022aa1n12x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  oab012aa1d18x5               g006(.a(new_n99), .b(new_n101), .c(new_n100), .out0(new_n102));
  tech160nm_fixorc02aa1n02p5x5 g007(.a(\a[4] ), .b(\b[3] ), .out0(new_n103));
  nor002aa1n03x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand42aa1n16x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  norb02aa1n03x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nanp03aa1n09x5               g011(.a(new_n102), .b(new_n103), .c(new_n106), .o1(new_n107));
  aoi112aa1n03x5               g012(.a(\b[2] ), .b(\a[3] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n108));
  oab012aa1n04x5               g013(.a(new_n108), .b(\a[4] ), .c(\b[3] ), .out0(new_n109));
  nor002aa1n08x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nand02aa1d24x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor002aa1n08x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand02aa1d24x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nano23aa1n09x5               g018(.a(new_n110), .b(new_n112), .c(new_n113), .d(new_n111), .out0(new_n114));
  nor042aa1n04x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nand22aa1n12x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nor042aa1d18x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nand02aa1d06x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  nano23aa1n03x7               g023(.a(new_n115), .b(new_n117), .c(new_n118), .d(new_n116), .out0(new_n119));
  nanp02aa1n02x5               g024(.a(new_n119), .b(new_n114), .o1(new_n120));
  ao0012aa1n03x7               g025(.a(new_n115), .b(new_n117), .c(new_n116), .o(new_n121));
  ao0012aa1n02x5               g026(.a(new_n110), .b(new_n112), .c(new_n111), .o(new_n122));
  tech160nm_fiaoi012aa1n03p5x5 g027(.a(new_n122), .b(new_n114), .c(new_n121), .o1(new_n123));
  aoai13aa1n12x5               g028(.a(new_n123), .b(new_n120), .c(new_n107), .d(new_n109), .o1(new_n124));
  xorc02aa1n12x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoi012aa1n02x5               g030(.a(new_n98), .b(new_n124), .c(new_n125), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  tech160nm_fixnrc02aa1n04x5   g032(.a(\b[9] ), .b(\a[10] ), .out0(new_n128));
  norb02aa1n02x5               g033(.a(new_n125), .b(new_n128), .out0(new_n129));
  aob012aa1n02x5               g034(.a(new_n98), .b(\b[9] ), .c(\a[10] ), .out0(new_n130));
  oaib12aa1n09x5               g035(.a(new_n130), .b(\b[9] ), .c(new_n97), .out0(new_n131));
  tech160nm_fiao0012aa1n02p5x5 g036(.a(new_n131), .b(new_n124), .c(new_n129), .o(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n06x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand42aa1d28x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  aoi012aa1n02x5               g040(.a(new_n134), .b(new_n132), .c(new_n135), .o1(new_n136));
  xnrb03aa1n02x5               g041(.a(new_n136), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor042aa1n04x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand42aa1n20x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nano23aa1d15x5               g044(.a(new_n134), .b(new_n138), .c(new_n139), .d(new_n135), .out0(new_n140));
  tech160nm_fiao0012aa1n02p5x5 g045(.a(new_n138), .b(new_n134), .c(new_n139), .o(new_n141));
  tech160nm_fiao0012aa1n02p5x5 g046(.a(new_n141), .b(new_n140), .c(new_n131), .o(new_n142));
  nanb03aa1d24x5               g047(.a(new_n128), .b(new_n140), .c(new_n125), .out0(new_n143));
  inv000aa1d42x5               g048(.a(new_n143), .o1(new_n144));
  xnrc02aa1n12x5               g049(.a(\b[12] ), .b(\a[13] ), .out0(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  aoai13aa1n02x5               g051(.a(new_n146), .b(new_n142), .c(new_n124), .d(new_n144), .o1(new_n147));
  aoi112aa1n02x5               g052(.a(new_n142), .b(new_n146), .c(new_n124), .d(new_n144), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n147), .b(new_n148), .out0(\s[13] ));
  orn002aa1n02x5               g054(.a(\a[13] ), .b(\b[12] ), .o(new_n150));
  xnrc02aa1n12x5               g055(.a(\b[13] ), .b(\a[14] ), .out0(new_n151));
  xobna2aa1n03x5               g056(.a(new_n151), .b(new_n147), .c(new_n150), .out0(\s[14] ));
  nona32aa1n09x5               g057(.a(new_n124), .b(new_n151), .c(new_n145), .d(new_n143), .out0(new_n153));
  nor042aa1n06x5               g058(.a(new_n151), .b(new_n145), .o1(new_n154));
  aoai13aa1n04x5               g059(.a(new_n154), .b(new_n141), .c(new_n140), .d(new_n131), .o1(new_n155));
  oao003aa1n02x5               g060(.a(\a[14] ), .b(\b[13] ), .c(new_n150), .carry(new_n156));
  nand02aa1d06x5               g061(.a(new_n155), .b(new_n156), .o1(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  xnrc02aa1n12x5               g063(.a(\b[14] ), .b(\a[15] ), .out0(new_n159));
  xobna2aa1n03x5               g064(.a(new_n159), .b(new_n153), .c(new_n158), .out0(\s[15] ));
  nor042aa1n03x5               g065(.a(\b[14] ), .b(\a[15] ), .o1(new_n161));
  tech160nm_fiao0012aa1n02p5x5 g066(.a(new_n159), .b(new_n153), .c(new_n158), .o(new_n162));
  xnrc02aa1n12x5               g067(.a(\b[15] ), .b(\a[16] ), .out0(new_n163));
  oaib12aa1n03x5               g068(.a(new_n163), .b(new_n161), .c(new_n162), .out0(new_n164));
  nona22aa1n02x4               g069(.a(new_n162), .b(new_n163), .c(new_n161), .out0(new_n165));
  nanp02aa1n03x5               g070(.a(new_n164), .b(new_n165), .o1(\s[16] ));
  inv000aa1d42x5               g071(.a(\a[17] ), .o1(new_n167));
  nor042aa1n04x5               g072(.a(new_n163), .b(new_n159), .o1(new_n168));
  nano22aa1d15x5               g073(.a(new_n143), .b(new_n154), .c(new_n168), .out0(new_n169));
  inv000aa1n02x5               g074(.a(new_n168), .o1(new_n170));
  tech160nm_fiaoi012aa1n04x5   g075(.a(new_n170), .b(new_n155), .c(new_n156), .o1(new_n171));
  inv000aa1d42x5               g076(.a(\a[16] ), .o1(new_n172));
  inv000aa1d42x5               g077(.a(\b[15] ), .o1(new_n173));
  oaoi03aa1n12x5               g078(.a(new_n172), .b(new_n173), .c(new_n161), .o1(new_n174));
  inv000aa1n02x5               g079(.a(new_n174), .o1(new_n175));
  aoi112aa1n09x5               g080(.a(new_n171), .b(new_n175), .c(new_n124), .d(new_n169), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[16] ), .c(new_n167), .out0(\s[17] ));
  oaoi03aa1n03x5               g082(.a(\a[17] ), .b(\b[16] ), .c(new_n176), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv040aa1d32x5               g084(.a(\a[18] ), .o1(new_n180));
  xroi22aa1d06x4               g085(.a(new_n167), .b(\b[16] ), .c(new_n180), .d(\b[17] ), .out0(new_n181));
  inv040aa1n03x5               g086(.a(new_n181), .o1(new_n182));
  aoi112aa1n06x5               g087(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n183));
  aoib12aa1n09x5               g088(.a(new_n183), .b(new_n180), .c(\b[17] ), .out0(new_n184));
  tech160nm_fioai012aa1n05x5   g089(.a(new_n184), .b(new_n176), .c(new_n182), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g091(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g092(.a(\b[18] ), .b(\a[19] ), .o1(new_n188));
  nand02aa1n06x5               g093(.a(\b[18] ), .b(\a[19] ), .o1(new_n189));
  norb02aa1n02x5               g094(.a(new_n189), .b(new_n188), .out0(new_n190));
  nor002aa1d32x5               g095(.a(\b[19] ), .b(\a[20] ), .o1(new_n191));
  nand22aa1n12x5               g096(.a(\b[19] ), .b(\a[20] ), .o1(new_n192));
  nanb02aa1n02x5               g097(.a(new_n191), .b(new_n192), .out0(new_n193));
  aoai13aa1n03x5               g098(.a(new_n193), .b(new_n188), .c(new_n185), .d(new_n190), .o1(new_n194));
  nanp02aa1n04x5               g099(.a(new_n124), .b(new_n169), .o1(new_n195));
  nand02aa1d06x5               g100(.a(new_n157), .b(new_n168), .o1(new_n196));
  nanp03aa1d12x5               g101(.a(new_n195), .b(new_n196), .c(new_n174), .o1(new_n197));
  nor042aa1n03x5               g102(.a(\b[16] ), .b(\a[17] ), .o1(new_n198));
  aob012aa1n02x5               g103(.a(new_n198), .b(\b[17] ), .c(\a[18] ), .out0(new_n199));
  oaib12aa1n09x5               g104(.a(new_n199), .b(\b[17] ), .c(new_n180), .out0(new_n200));
  aoai13aa1n03x5               g105(.a(new_n190), .b(new_n200), .c(new_n197), .d(new_n181), .o1(new_n201));
  nona22aa1n03x5               g106(.a(new_n201), .b(new_n193), .c(new_n188), .out0(new_n202));
  nanp02aa1n03x5               g107(.a(new_n194), .b(new_n202), .o1(\s[20] ));
  nona23aa1n09x5               g108(.a(new_n192), .b(new_n189), .c(new_n188), .d(new_n191), .out0(new_n204));
  ao0012aa1n03x7               g109(.a(new_n191), .b(new_n188), .c(new_n192), .o(new_n205));
  oabi12aa1n18x5               g110(.a(new_n205), .b(new_n204), .c(new_n184), .out0(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  norb02aa1n03x5               g112(.a(new_n181), .b(new_n204), .out0(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  tech160nm_fioai012aa1n05x5   g114(.a(new_n207), .b(new_n176), .c(new_n209), .o1(new_n210));
  xorb03aa1n02x5               g115(.a(new_n210), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g116(.a(\b[20] ), .b(\a[21] ), .o1(new_n212));
  xnrc02aa1n12x5               g117(.a(\b[20] ), .b(\a[21] ), .out0(new_n213));
  inv000aa1d42x5               g118(.a(new_n213), .o1(new_n214));
  xnrc02aa1n12x5               g119(.a(\b[21] ), .b(\a[22] ), .out0(new_n215));
  aoai13aa1n03x5               g120(.a(new_n215), .b(new_n212), .c(new_n210), .d(new_n214), .o1(new_n216));
  aoai13aa1n03x5               g121(.a(new_n214), .b(new_n206), .c(new_n197), .d(new_n208), .o1(new_n217));
  nona22aa1n03x5               g122(.a(new_n217), .b(new_n215), .c(new_n212), .out0(new_n218));
  nanp02aa1n03x5               g123(.a(new_n216), .b(new_n218), .o1(\s[22] ));
  nano23aa1n06x5               g124(.a(new_n188), .b(new_n191), .c(new_n192), .d(new_n189), .out0(new_n220));
  nor042aa1n06x5               g125(.a(new_n215), .b(new_n213), .o1(new_n221));
  nano22aa1n12x5               g126(.a(new_n182), .b(new_n221), .c(new_n220), .out0(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  inv000aa1d42x5               g128(.a(\a[22] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\b[21] ), .o1(new_n225));
  oaoi03aa1n12x5               g130(.a(new_n224), .b(new_n225), .c(new_n212), .o1(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  aoi012aa1n09x5               g132(.a(new_n227), .b(new_n206), .c(new_n221), .o1(new_n228));
  tech160nm_fioai012aa1n05x5   g133(.a(new_n228), .b(new_n176), .c(new_n223), .o1(new_n229));
  xorb03aa1n02x5               g134(.a(new_n229), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g135(.a(\b[22] ), .b(\a[23] ), .o1(new_n231));
  xorc02aa1n12x5               g136(.a(\a[23] ), .b(\b[22] ), .out0(new_n232));
  xnrc02aa1n12x5               g137(.a(\b[23] ), .b(\a[24] ), .out0(new_n233));
  aoai13aa1n03x5               g138(.a(new_n233), .b(new_n231), .c(new_n229), .d(new_n232), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n228), .o1(new_n235));
  aoai13aa1n03x5               g140(.a(new_n232), .b(new_n235), .c(new_n197), .d(new_n222), .o1(new_n236));
  nona22aa1n03x5               g141(.a(new_n236), .b(new_n233), .c(new_n231), .out0(new_n237));
  nanp02aa1n03x5               g142(.a(new_n234), .b(new_n237), .o1(\s[24] ));
  norb02aa1n02x7               g143(.a(new_n232), .b(new_n233), .out0(new_n239));
  inv040aa1n02x5               g144(.a(new_n239), .o1(new_n240));
  nano32aa1n03x7               g145(.a(new_n240), .b(new_n181), .c(new_n221), .d(new_n220), .out0(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  aoai13aa1n06x5               g147(.a(new_n221), .b(new_n205), .c(new_n220), .d(new_n200), .o1(new_n243));
  aoi112aa1n02x5               g148(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n244));
  oab012aa1n04x5               g149(.a(new_n244), .b(\a[24] ), .c(\b[23] ), .out0(new_n245));
  aoai13aa1n12x5               g150(.a(new_n245), .b(new_n240), .c(new_n243), .d(new_n226), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n246), .o1(new_n247));
  tech160nm_fioai012aa1n05x5   g152(.a(new_n247), .b(new_n176), .c(new_n242), .o1(new_n248));
  xorb03aa1n02x5               g153(.a(new_n248), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g154(.a(\b[24] ), .b(\a[25] ), .o1(new_n250));
  xorc02aa1n12x5               g155(.a(\a[25] ), .b(\b[24] ), .out0(new_n251));
  tech160nm_fixnrc02aa1n05x5   g156(.a(\b[25] ), .b(\a[26] ), .out0(new_n252));
  aoai13aa1n03x5               g157(.a(new_n252), .b(new_n250), .c(new_n248), .d(new_n251), .o1(new_n253));
  aoai13aa1n03x5               g158(.a(new_n251), .b(new_n246), .c(new_n197), .d(new_n241), .o1(new_n254));
  nona22aa1n03x5               g159(.a(new_n254), .b(new_n252), .c(new_n250), .out0(new_n255));
  nanp02aa1n03x5               g160(.a(new_n253), .b(new_n255), .o1(\s[26] ));
  norb02aa1n02x5               g161(.a(new_n251), .b(new_n252), .out0(new_n257));
  inv000aa1n02x5               g162(.a(new_n257), .o1(new_n258));
  nona22aa1n03x5               g163(.a(new_n222), .b(new_n258), .c(new_n240), .out0(new_n259));
  inv000aa1d42x5               g164(.a(\a[26] ), .o1(new_n260));
  inv000aa1d42x5               g165(.a(\b[25] ), .o1(new_n261));
  oaoi03aa1n06x5               g166(.a(new_n260), .b(new_n261), .c(new_n250), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  aoi012aa1n06x5               g168(.a(new_n263), .b(new_n246), .c(new_n257), .o1(new_n264));
  oai012aa1n06x5               g169(.a(new_n264), .b(new_n176), .c(new_n259), .o1(new_n265));
  xorb03aa1n03x5               g170(.a(new_n265), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g171(.a(\b[26] ), .b(\a[27] ), .o1(new_n267));
  xorc02aa1n02x5               g172(.a(\a[27] ), .b(\b[26] ), .out0(new_n268));
  xnrc02aa1n02x5               g173(.a(\b[27] ), .b(\a[28] ), .out0(new_n269));
  aoai13aa1n03x5               g174(.a(new_n269), .b(new_n267), .c(new_n265), .d(new_n268), .o1(new_n270));
  inv040aa1n02x5               g175(.a(new_n259), .o1(new_n271));
  aoai13aa1n06x5               g176(.a(new_n239), .b(new_n227), .c(new_n206), .d(new_n221), .o1(new_n272));
  aoai13aa1n04x5               g177(.a(new_n262), .b(new_n258), .c(new_n272), .d(new_n245), .o1(new_n273));
  aoai13aa1n02x5               g178(.a(new_n268), .b(new_n273), .c(new_n197), .d(new_n271), .o1(new_n274));
  nona22aa1n02x4               g179(.a(new_n274), .b(new_n269), .c(new_n267), .out0(new_n275));
  nanp02aa1n03x5               g180(.a(new_n270), .b(new_n275), .o1(\s[28] ));
  norb02aa1n02x5               g181(.a(new_n268), .b(new_n269), .out0(new_n277));
  aoai13aa1n02x5               g182(.a(new_n277), .b(new_n273), .c(new_n197), .d(new_n271), .o1(new_n278));
  aob012aa1n03x5               g183(.a(new_n267), .b(\b[27] ), .c(\a[28] ), .out0(new_n279));
  oa0012aa1n12x5               g184(.a(new_n279), .b(\b[27] ), .c(\a[28] ), .o(new_n280));
  inv000aa1d42x5               g185(.a(new_n280), .o1(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[28] ), .b(\a[29] ), .out0(new_n282));
  nona22aa1n02x4               g187(.a(new_n278), .b(new_n281), .c(new_n282), .out0(new_n283));
  aoai13aa1n03x5               g188(.a(new_n282), .b(new_n281), .c(new_n265), .d(new_n277), .o1(new_n284));
  nanp02aa1n03x5               g189(.a(new_n284), .b(new_n283), .o1(\s[29] ));
  xorb03aa1n02x5               g190(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g191(.a(new_n268), .b(new_n282), .c(new_n269), .out0(new_n287));
  oaoi03aa1n09x5               g192(.a(\a[29] ), .b(\b[28] ), .c(new_n280), .o1(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[29] ), .b(\a[30] ), .out0(new_n289));
  aoai13aa1n03x5               g194(.a(new_n289), .b(new_n288), .c(new_n265), .d(new_n287), .o1(new_n290));
  aoai13aa1n02x5               g195(.a(new_n287), .b(new_n273), .c(new_n197), .d(new_n271), .o1(new_n291));
  nona22aa1n02x4               g196(.a(new_n291), .b(new_n288), .c(new_n289), .out0(new_n292));
  nanp02aa1n03x5               g197(.a(new_n290), .b(new_n292), .o1(\s[30] ));
  norb02aa1n02x5               g198(.a(new_n287), .b(new_n289), .out0(new_n294));
  aoai13aa1n02x5               g199(.a(new_n294), .b(new_n273), .c(new_n197), .d(new_n271), .o1(new_n295));
  nanb02aa1n02x5               g200(.a(new_n289), .b(new_n288), .out0(new_n296));
  oai012aa1n02x5               g201(.a(new_n296), .b(\b[29] ), .c(\a[30] ), .o1(new_n297));
  xnrc02aa1n02x5               g202(.a(\b[30] ), .b(\a[31] ), .out0(new_n298));
  nona22aa1n02x4               g203(.a(new_n295), .b(new_n297), .c(new_n298), .out0(new_n299));
  aoai13aa1n03x5               g204(.a(new_n298), .b(new_n297), .c(new_n265), .d(new_n294), .o1(new_n300));
  nanp02aa1n03x5               g205(.a(new_n300), .b(new_n299), .o1(\s[31] ));
  xorb03aa1n02x5               g206(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  aoi012aa1n02x5               g207(.a(new_n104), .b(new_n102), .c(new_n105), .o1(new_n303));
  xnrb03aa1n02x5               g208(.a(new_n303), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  nanp02aa1n02x5               g209(.a(new_n107), .b(new_n109), .o1(new_n305));
  xorb03aa1n02x5               g210(.a(new_n305), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g211(.a(new_n117), .b(new_n305), .c(new_n118), .o1(new_n307));
  xnrb03aa1n02x5               g212(.a(new_n307), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g213(.a(new_n121), .b(new_n305), .c(new_n119), .o(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g215(.a(new_n112), .b(new_n309), .c(new_n113), .o1(new_n311));
  xnrb03aa1n02x5               g216(.a(new_n311), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g217(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


