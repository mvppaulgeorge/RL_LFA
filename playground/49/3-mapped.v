// Benchmark "adder" written by ABC on Thu Jul 18 13:17:26 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n314, new_n317, new_n319, new_n321;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand42aa1n03x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand22aa1n12x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nor042aa1n06x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  oai012aa1n12x5               g004(.a(new_n97), .b(new_n99), .c(new_n98), .o1(new_n100));
  tech160nm_fixnrc02aa1n04x5   g005(.a(\b[3] ), .b(\a[4] ), .out0(new_n101));
  nor022aa1n08x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nand42aa1n06x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanb02aa1n06x5               g008(.a(new_n102), .b(new_n103), .out0(new_n104));
  inv000aa1d42x5               g009(.a(\a[3] ), .o1(new_n105));
  nanb02aa1n12x5               g010(.a(\b[2] ), .b(new_n105), .out0(new_n106));
  oao003aa1n06x5               g011(.a(\a[4] ), .b(\b[3] ), .c(new_n106), .carry(new_n107));
  oai013aa1n09x5               g012(.a(new_n107), .b(new_n101), .c(new_n100), .d(new_n104), .o1(new_n108));
  xnrc02aa1n12x5               g013(.a(\b[7] ), .b(\a[8] ), .out0(new_n109));
  xnrc02aa1n12x5               g014(.a(\b[6] ), .b(\a[7] ), .out0(new_n110));
  nor022aa1n08x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  nand22aa1n12x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nor002aa1d32x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nand42aa1n03x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nona23aa1n09x5               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  nor043aa1n04x5               g020(.a(new_n115), .b(new_n110), .c(new_n109), .o1(new_n116));
  tech160nm_fiaoi012aa1n05x5   g021(.a(new_n111), .b(new_n113), .c(new_n112), .o1(new_n117));
  inv000aa1d42x5               g022(.a(\a[8] ), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\b[7] ), .o1(new_n119));
  norp02aa1n02x5               g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  oaoi03aa1n02x5               g025(.a(new_n118), .b(new_n119), .c(new_n120), .o1(new_n121));
  oai013aa1n03x5               g026(.a(new_n121), .b(new_n110), .c(new_n109), .d(new_n117), .o1(new_n122));
  aoi012aa1n02x5               g027(.a(new_n122), .b(new_n108), .c(new_n116), .o1(new_n123));
  oaoi03aa1n02x5               g028(.a(\a[9] ), .b(\b[8] ), .c(new_n123), .o1(new_n124));
  xorb03aa1n02x5               g029(.a(new_n124), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nanp02aa1n02x5               g030(.a(new_n108), .b(new_n116), .o1(new_n126));
  norp03aa1n02x5               g031(.a(new_n109), .b(new_n110), .c(new_n117), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n121), .b(new_n127), .out0(new_n128));
  nor002aa1n06x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  nor022aa1n16x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nand42aa1n20x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  aoi012aa1n02x7               g036(.a(new_n130), .b(new_n129), .c(new_n131), .o1(new_n132));
  nand42aa1d28x5               g037(.a(\b[8] ), .b(\a[9] ), .o1(new_n133));
  nano23aa1d15x5               g038(.a(new_n129), .b(new_n130), .c(new_n131), .d(new_n133), .out0(new_n134));
  inv000aa1d42x5               g039(.a(new_n134), .o1(new_n135));
  aoai13aa1n02x5               g040(.a(new_n132), .b(new_n135), .c(new_n126), .d(new_n128), .o1(new_n136));
  xorb03aa1n02x5               g041(.a(new_n136), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  inv000aa1d42x5               g042(.a(\a[12] ), .o1(new_n138));
  nor002aa1n03x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nand42aa1n02x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  aoi012aa1n02x5               g045(.a(new_n139), .b(new_n136), .c(new_n140), .o1(new_n141));
  xorb03aa1n02x5               g046(.a(new_n141), .b(\b[11] ), .c(new_n138), .out0(\s[12] ));
  norp02aa1n04x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand22aa1n02x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nona23aa1n09x5               g049(.a(new_n144), .b(new_n140), .c(new_n139), .d(new_n143), .out0(new_n145));
  ao0012aa1n03x7               g050(.a(new_n143), .b(new_n139), .c(new_n144), .o(new_n146));
  oab012aa1n02x4               g051(.a(new_n146), .b(new_n145), .c(new_n132), .out0(new_n147));
  norb02aa1n02x5               g052(.a(new_n134), .b(new_n145), .out0(new_n148));
  aoai13aa1n06x5               g053(.a(new_n148), .b(new_n122), .c(new_n108), .d(new_n116), .o1(new_n149));
  nanp02aa1n03x5               g054(.a(new_n149), .b(new_n147), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g056(.a(\a[14] ), .o1(new_n152));
  nor002aa1n03x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nand42aa1d28x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  aoi012aa1n03x5               g059(.a(new_n153), .b(new_n150), .c(new_n154), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[13] ), .c(new_n152), .out0(\s[14] ));
  nor042aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nand42aa1n08x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  nona23aa1n02x4               g063(.a(new_n158), .b(new_n154), .c(new_n153), .d(new_n157), .out0(new_n159));
  oabi12aa1n06x5               g064(.a(new_n146), .b(new_n145), .c(new_n132), .out0(new_n160));
  nano23aa1n06x5               g065(.a(new_n153), .b(new_n157), .c(new_n158), .d(new_n154), .out0(new_n161));
  tech160nm_fiaoi012aa1n05x5   g066(.a(new_n157), .b(new_n153), .c(new_n158), .o1(new_n162));
  inv000aa1n02x5               g067(.a(new_n162), .o1(new_n163));
  aoi012aa1n02x5               g068(.a(new_n163), .b(new_n160), .c(new_n161), .o1(new_n164));
  tech160nm_fioai012aa1n05x5   g069(.a(new_n164), .b(new_n149), .c(new_n159), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n04x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nand42aa1d28x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  nor002aa1n03x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nanp02aa1n24x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  inv000aa1d42x5               g077(.a(new_n172), .o1(new_n173));
  aoai13aa1n02x5               g078(.a(new_n173), .b(new_n167), .c(new_n165), .d(new_n169), .o1(new_n174));
  aoi112aa1n02x5               g079(.a(new_n167), .b(new_n173), .c(new_n165), .d(new_n168), .o1(new_n175));
  nanb02aa1n03x5               g080(.a(new_n175), .b(new_n174), .out0(\s[16] ));
  nano23aa1n06x5               g081(.a(new_n139), .b(new_n143), .c(new_n144), .d(new_n140), .out0(new_n177));
  nano23aa1n03x5               g082(.a(new_n167), .b(new_n170), .c(new_n171), .d(new_n168), .out0(new_n178));
  nand02aa1n02x5               g083(.a(new_n178), .b(new_n161), .o1(new_n179));
  nano22aa1n03x7               g084(.a(new_n179), .b(new_n134), .c(new_n177), .out0(new_n180));
  aoai13aa1n12x5               g085(.a(new_n180), .b(new_n122), .c(new_n108), .d(new_n116), .o1(new_n181));
  aoi012aa1n02x5               g086(.a(new_n170), .b(new_n167), .c(new_n171), .o1(new_n182));
  aob012aa1n02x5               g087(.a(new_n182), .b(new_n178), .c(new_n163), .out0(new_n183));
  aoib12aa1n12x5               g088(.a(new_n183), .b(new_n160), .c(new_n179), .out0(new_n184));
  nor042aa1d18x5               g089(.a(\b[16] ), .b(\a[17] ), .o1(new_n185));
  nand42aa1n03x5               g090(.a(\b[16] ), .b(\a[17] ), .o1(new_n186));
  norb02aa1n03x5               g091(.a(new_n186), .b(new_n185), .out0(new_n187));
  xnbna2aa1n03x5               g092(.a(new_n187), .b(new_n181), .c(new_n184), .out0(\s[17] ));
  nanp02aa1n06x5               g093(.a(new_n181), .b(new_n184), .o1(new_n189));
  aoi012aa1n06x5               g094(.a(new_n185), .b(new_n189), .c(new_n187), .o1(new_n190));
  inv040aa1d32x5               g095(.a(\a[18] ), .o1(new_n191));
  inv040aa1d28x5               g096(.a(\b[17] ), .o1(new_n192));
  nand02aa1d20x5               g097(.a(new_n192), .b(new_n191), .o1(new_n193));
  nand02aa1d28x5               g098(.a(\b[17] ), .b(\a[18] ), .o1(new_n194));
  xnbna2aa1n03x5               g099(.a(new_n190), .b(new_n194), .c(new_n193), .out0(\s[18] ));
  aob012aa1d24x5               g100(.a(new_n193), .b(new_n185), .c(new_n194), .out0(new_n196));
  inv000aa1d42x5               g101(.a(new_n196), .o1(new_n197));
  nano32aa1n06x5               g102(.a(new_n185), .b(new_n194), .c(new_n186), .d(new_n193), .out0(new_n198));
  inv000aa1d42x5               g103(.a(new_n198), .o1(new_n199));
  aoai13aa1n06x5               g104(.a(new_n197), .b(new_n199), .c(new_n181), .d(new_n184), .o1(new_n200));
  xorb03aa1n02x5               g105(.a(new_n200), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g106(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nanp02aa1n02x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  norb02aa1n06x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  nor042aa1n06x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanp02aa1n09x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  norb02aa1n15x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  aoai13aa1n03x5               g114(.a(new_n209), .b(new_n203), .c(new_n200), .d(new_n205), .o1(new_n210));
  nand02aa1n03x5               g115(.a(new_n200), .b(new_n205), .o1(new_n211));
  nona22aa1n02x5               g116(.a(new_n211), .b(new_n209), .c(new_n203), .out0(new_n212));
  nanp02aa1n03x5               g117(.a(new_n212), .b(new_n210), .o1(\s[20] ));
  nanp03aa1d12x5               g118(.a(new_n196), .b(new_n205), .c(new_n208), .o1(new_n214));
  aoi012aa1n09x5               g119(.a(new_n206), .b(new_n203), .c(new_n207), .o1(new_n215));
  nand02aa1n04x5               g120(.a(new_n214), .b(new_n215), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  nano23aa1n02x5               g122(.a(new_n203), .b(new_n206), .c(new_n207), .d(new_n204), .out0(new_n218));
  inv000aa1n03x5               g123(.a(new_n218), .o1(new_n219));
  nano32aa1n03x7               g124(.a(new_n219), .b(new_n187), .c(new_n193), .d(new_n194), .out0(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  aoai13aa1n06x5               g126(.a(new_n217), .b(new_n221), .c(new_n181), .d(new_n184), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1d18x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  nand42aa1d28x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  norb02aa1n02x5               g130(.a(new_n225), .b(new_n224), .out0(new_n226));
  nor022aa1n16x5               g131(.a(\b[21] ), .b(\a[22] ), .o1(new_n227));
  nand02aa1d28x5               g132(.a(\b[21] ), .b(\a[22] ), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n228), .b(new_n227), .out0(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  aoai13aa1n03x5               g135(.a(new_n230), .b(new_n224), .c(new_n222), .d(new_n226), .o1(new_n231));
  nand02aa1n03x5               g136(.a(new_n222), .b(new_n226), .o1(new_n232));
  nona22aa1n02x5               g137(.a(new_n232), .b(new_n230), .c(new_n224), .out0(new_n233));
  nanp02aa1n03x5               g138(.a(new_n233), .b(new_n231), .o1(\s[22] ));
  nano23aa1d15x5               g139(.a(new_n224), .b(new_n227), .c(new_n228), .d(new_n225), .out0(new_n235));
  nanp03aa1n02x5               g140(.a(new_n198), .b(new_n218), .c(new_n235), .o1(new_n236));
  ao0012aa1n03x7               g141(.a(new_n227), .b(new_n224), .c(new_n228), .o(new_n237));
  aoi012aa1n03x5               g142(.a(new_n237), .b(new_n216), .c(new_n235), .o1(new_n238));
  aoai13aa1n04x5               g143(.a(new_n238), .b(new_n236), .c(new_n181), .d(new_n184), .o1(new_n239));
  xorb03aa1n02x5               g144(.a(new_n239), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n06x5               g145(.a(\b[22] ), .b(\a[23] ), .o1(new_n241));
  nand02aa1d16x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  norb02aa1n02x5               g147(.a(new_n242), .b(new_n241), .out0(new_n243));
  nor042aa1n06x5               g148(.a(\b[23] ), .b(\a[24] ), .o1(new_n244));
  nand02aa1d16x5               g149(.a(\b[23] ), .b(\a[24] ), .o1(new_n245));
  nanb02aa1n02x5               g150(.a(new_n244), .b(new_n245), .out0(new_n246));
  aoai13aa1n03x5               g151(.a(new_n246), .b(new_n241), .c(new_n239), .d(new_n243), .o1(new_n247));
  nand02aa1d04x5               g152(.a(new_n239), .b(new_n243), .o1(new_n248));
  nona22aa1n03x5               g153(.a(new_n248), .b(new_n246), .c(new_n241), .out0(new_n249));
  nanp02aa1n03x5               g154(.a(new_n249), .b(new_n247), .o1(\s[24] ));
  nano23aa1n09x5               g155(.a(new_n241), .b(new_n244), .c(new_n245), .d(new_n242), .out0(new_n251));
  nand22aa1n09x5               g156(.a(new_n251), .b(new_n235), .o1(new_n252));
  ao0012aa1n03x5               g157(.a(new_n244), .b(new_n241), .c(new_n245), .o(new_n253));
  aoi012aa1n06x5               g158(.a(new_n253), .b(new_n251), .c(new_n237), .o1(new_n254));
  aoai13aa1n12x5               g159(.a(new_n254), .b(new_n252), .c(new_n214), .d(new_n215), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n252), .o1(new_n257));
  nand02aa1d04x5               g162(.a(new_n220), .b(new_n257), .o1(new_n258));
  aoai13aa1n04x5               g163(.a(new_n256), .b(new_n258), .c(new_n181), .d(new_n184), .o1(new_n259));
  xorb03aa1n02x5               g164(.a(new_n259), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g165(.a(\b[24] ), .b(\a[25] ), .o1(new_n261));
  tech160nm_fixorc02aa1n03p5x5 g166(.a(\a[25] ), .b(\b[24] ), .out0(new_n262));
  tech160nm_fixnrc02aa1n05x5   g167(.a(\b[25] ), .b(\a[26] ), .out0(new_n263));
  aoai13aa1n03x5               g168(.a(new_n263), .b(new_n261), .c(new_n259), .d(new_n262), .o1(new_n264));
  nand42aa1n02x5               g169(.a(new_n259), .b(new_n262), .o1(new_n265));
  nona22aa1n02x4               g170(.a(new_n265), .b(new_n263), .c(new_n261), .out0(new_n266));
  nanp02aa1n03x5               g171(.a(new_n266), .b(new_n264), .o1(\s[26] ));
  norb02aa1n03x4               g172(.a(new_n262), .b(new_n263), .out0(new_n268));
  nano32aa1n03x7               g173(.a(new_n252), .b(new_n198), .c(new_n268), .d(new_n218), .out0(new_n269));
  inv020aa1n02x5               g174(.a(new_n269), .o1(new_n270));
  inv000aa1d42x5               g175(.a(\a[26] ), .o1(new_n271));
  inv000aa1d42x5               g176(.a(\b[25] ), .o1(new_n272));
  oaoi03aa1n02x5               g177(.a(new_n271), .b(new_n272), .c(new_n261), .o1(new_n273));
  inv000aa1n02x5               g178(.a(new_n273), .o1(new_n274));
  tech160nm_fiaoi012aa1n05x5   g179(.a(new_n274), .b(new_n255), .c(new_n268), .o1(new_n275));
  aoai13aa1n06x5               g180(.a(new_n275), .b(new_n270), .c(new_n181), .d(new_n184), .o1(new_n276));
  xorb03aa1n03x5               g181(.a(new_n276), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nor042aa1n03x5               g182(.a(\b[26] ), .b(\a[27] ), .o1(new_n278));
  xorc02aa1n02x5               g183(.a(\a[27] ), .b(\b[26] ), .out0(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[27] ), .b(\a[28] ), .out0(new_n280));
  aoai13aa1n03x5               g185(.a(new_n280), .b(new_n278), .c(new_n276), .d(new_n279), .o1(new_n281));
  nanp02aa1n09x5               g186(.a(new_n255), .b(new_n268), .o1(new_n282));
  nanp02aa1n06x5               g187(.a(new_n282), .b(new_n273), .o1(new_n283));
  aoai13aa1n03x5               g188(.a(new_n279), .b(new_n283), .c(new_n189), .d(new_n269), .o1(new_n284));
  nona22aa1n03x5               g189(.a(new_n284), .b(new_n280), .c(new_n278), .out0(new_n285));
  nanp02aa1n03x5               g190(.a(new_n281), .b(new_n285), .o1(\s[28] ));
  inv000aa1d42x5               g191(.a(\a[28] ), .o1(new_n287));
  inv000aa1d42x5               g192(.a(\b[27] ), .o1(new_n288));
  oaoi03aa1n09x5               g193(.a(new_n287), .b(new_n288), .c(new_n278), .o1(new_n289));
  inv000aa1d42x5               g194(.a(new_n289), .o1(new_n290));
  norb02aa1n02x5               g195(.a(new_n279), .b(new_n280), .out0(new_n291));
  aoai13aa1n03x5               g196(.a(new_n291), .b(new_n283), .c(new_n189), .d(new_n269), .o1(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[28] ), .b(\a[29] ), .out0(new_n293));
  nona22aa1n03x5               g198(.a(new_n292), .b(new_n293), .c(new_n290), .out0(new_n294));
  aoai13aa1n03x5               g199(.a(new_n293), .b(new_n290), .c(new_n276), .d(new_n291), .o1(new_n295));
  nanp02aa1n03x5               g200(.a(new_n295), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g201(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  tech160nm_fioaoi03aa1n03p5x5 g202(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .o1(new_n298));
  norb03aa1n02x5               g203(.a(new_n279), .b(new_n293), .c(new_n280), .out0(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[29] ), .b(\a[30] ), .out0(new_n300));
  aoai13aa1n03x5               g205(.a(new_n300), .b(new_n298), .c(new_n276), .d(new_n299), .o1(new_n301));
  aoai13aa1n02x7               g206(.a(new_n299), .b(new_n283), .c(new_n189), .d(new_n269), .o1(new_n302));
  nona22aa1n02x5               g207(.a(new_n302), .b(new_n300), .c(new_n298), .out0(new_n303));
  nanp02aa1n03x5               g208(.a(new_n301), .b(new_n303), .o1(\s[30] ));
  norb02aa1n02x5               g209(.a(new_n299), .b(new_n300), .out0(new_n305));
  aoai13aa1n03x5               g210(.a(new_n305), .b(new_n283), .c(new_n189), .d(new_n269), .o1(new_n306));
  nanb02aa1n02x5               g211(.a(new_n300), .b(new_n298), .out0(new_n307));
  oai012aa1n02x5               g212(.a(new_n307), .b(\b[29] ), .c(\a[30] ), .o1(new_n308));
  xnrc02aa1n02x5               g213(.a(\b[30] ), .b(\a[31] ), .out0(new_n309));
  nona22aa1n03x5               g214(.a(new_n306), .b(new_n308), .c(new_n309), .out0(new_n310));
  aoai13aa1n03x5               g215(.a(new_n309), .b(new_n308), .c(new_n276), .d(new_n305), .o1(new_n311));
  nanp02aa1n03x5               g216(.a(new_n311), .b(new_n310), .o1(\s[31] ));
  xnbna2aa1n03x5               g217(.a(new_n100), .b(new_n103), .c(new_n106), .out0(\s[3] ));
  oaoi03aa1n02x5               g218(.a(\a[3] ), .b(\b[2] ), .c(new_n100), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g220(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g221(.a(new_n113), .b(new_n108), .c(new_n114), .o1(new_n317));
  xnrb03aa1n02x5               g222(.a(new_n317), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g223(.a(new_n117), .b(new_n115), .c(new_n108), .out0(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoib12aa1n02x5               g225(.a(new_n120), .b(new_n319), .c(new_n110), .out0(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[7] ), .c(new_n118), .out0(\s[8] ));
  xnrb03aa1n02x5               g227(.a(new_n123), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


